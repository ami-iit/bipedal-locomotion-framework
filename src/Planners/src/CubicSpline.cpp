/**
 * @file CubicSpline.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <algorithm>
#include <iostream>

#include <Eigen/Sparse>

#include <BipedalLocomotion/Planners/CubicSpline.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::Planners;

struct CubicSpline::Impl
{
    /**
     * Struct containing the boundary condtion
     */
    struct BoundaryConditions
    {
        Eigen::VectorXd velocity;
        Eigen::VectorXd acceleration;
    };

    /**
     * Description of a knot
     */
    struct Knot
    {
        Eigen::VectorXd position; /**< Knot position */
        Eigen::VectorXd velocity; /**< first derivative of spline computed at t@knot */
        Eigen::VectorXd acceleration; /**< second derivative of spline computed at t@knot */

        double timeInstant; /**< Knot time (it is an absolute time ) */
    };

    /**
     * Description of a 5-th polynomial. It contains the coefficients, the initial/final knots and
     * the duration of the sub-trajectory.
     */
    struct Polynomial
    {
        Eigen::VectorXd a0;
        Eigen::VectorXd a1;
        Eigen::VectorXd a2;
        Eigen::VectorXd a3;

        const Knot* initialPoint;
        const Knot* finalPoint;

        double duration;
    };

    BoundaryConditions initialCondition; /**< Initial condition */
    BoundaryConditions finalCondition; /**< Final condition */

    std::vector<Knot> knots; /**< Collection of all the knots */
    std::vector<Polynomial> polynomials; /**< Collection of all the polynomials */

    bool areCoefficientsComputed{false}; /**< If true the coefficients are computed and updated */

    SplineState currentTrajectory; /**< Current trajectory stored in the advance state */
    double advanceTimeStep{0.0}; /**< Time step of the advance interface. */
    double advanceCurrentTime{0.0}; /**< current time of the advance object. */

    /**
     * Reset a given knot with a time instant and a position
     */
    void resetKnot(const double& timeInstant,
                   Eigen::Ref<const Eigen::VectorXd> position,
                   Knot& knot);

    /**
     * Get the position at given time for a sub-trajectory
     */
    void getPositionAtTime(const double& t,
                           const Polynomial& poly,
                           Eigen::Ref<Eigen::VectorXd> position);

    /**
     * Get the velocity at given time for a sub-trajectory
     */
    void getVelocityAtTime(const double& t,
                           const Polynomial& poly,
                           Eigen::Ref<Eigen::VectorXd> velocity);

    /**
     * Get the acceleration at given time for a sub-trajectory
     */
    void getAccelerationAtTime(const double& t,
                               const Polynomial& poly,
                               Eigen::Ref<Eigen::VectorXd> acceleration);

    /**
     * Compute the duration of each phase
     */
    bool computePhasesDuration();

    /**
     * Set the boundary condition. This function is called before computing all the intermediate
     * velocities and accelerations.
     */
    bool setBoundaryVelocitiesAndAcceleration();

    /**
     * addTripletCurrentKnot is a helper function to generate a triplet containing a 2x2 matrix.
     * This matrix is used to compute the intermediate velocity and acceleration.
     */
    void addTripletCurrentKnot(const int& knotIndex,
                               const int& rowOffset,
                               const int& columnOffset,
                               std::vector<Eigen::Triplet<double>>& tripletList);

    /**
     * addTripletPreviousKnot is a helper function to generate a triplet containing a 2x2 matrix.
     * This matrix is used to compute the intermediate velocity and acceleration.
     */
    void addTripletPreviousKnot(const int& knotIndex,
                                const int& rowOffset,
                                const int& columnOffset,
                                std::vector<Eigen::Triplet<double>>& tripletList);
    /**
     * addTripletNextKnot is a helper function to generate a triplet containing a 2x2 matrix.
     * This matrix is used to compute the intermediate velocity and acceleration.
     */
    void addTripletNextKnot(const int& knotIndex,
                            const int& rowOffset,
                            const int& columnOffset,
                            std::vector<Eigen::Triplet<double>>& tripletList);

    /**
     * addKnownTermKnotPosition is a helper function to generate a 2-d vector,
     * This vector is the known term used to compute the intermediate velocity and acceleration.
     */
    void addKnownTermKnotPosition(const std::size_t& knotIndex,
                                  const std::size_t& coordinateIndex,
                                  Eigen::Ref<Eigen::VectorXd> b);

    /**
     * addKnownTermNextKnot is a helper function to generate a 2-d vector,
     * This vector is the known term used to compute the intermediate velocity and acceleration.
     */
    void addKnownTermNextKnot(const std::size_t& knotIndex,
                              const std::size_t& coordinateIndex,
                              Eigen::Ref<Eigen::VectorXd> b);

    /**
     * addKnownTermPreviousKnot is a helper function to generate a 2-d vector,
     * This vector is the known term used to compute the intermediate velocity and acceleration.
     */
    void addKnownTermPreviousKnot(const std::size_t& knotIndex,
                                  const std::size_t& coordinateIndex,
                                  Eigen::Ref<Eigen::VectorXd> b);

    /**
     * Compute the intermediate velocities
     */
    void computeIntermediateVelocities();

    /**
     * Function to set the coefficients of the polynomials
     */
    void setPolynomialCoefficients(Polynomial& poly);

    /**
     * This function is the entry point to compute the coefficients of the spline
     */
    bool computeCoefficients();
};

CubicSpline::CubicSpline()
{
    m_pimpl = std::make_unique<Impl>();
}

CubicSpline::~CubicSpline() = default;

bool CubicSpline::setInitialConditions(Eigen::Ref<const Eigen::VectorXd> velocity,
                                       Eigen::Ref<const Eigen::VectorXd> /**acceleration*/)
{
    m_pimpl->initialCondition.velocity = velocity;
    m_pimpl->initialCondition.acceleration = Eigen::VectorXd::Zero(velocity.size());

    // Set the initial state for the advance interface
    m_pimpl->currentTrajectory.velocity = velocity;
    m_pimpl->currentTrajectory.acceleration = Eigen::VectorXd::Zero(velocity.size());

    // The initial conditions changed. The coefficients are outdated.
    m_pimpl->areCoefficientsComputed = false;

    if (m_pimpl->finalCondition.velocity.size() != 0 && !m_pimpl->knots.empty())
    {
        return this->evaluatePoint(m_pimpl->knots.front().timeInstant, m_pimpl->currentTrajectory);
    }

    return true;
}

bool CubicSpline::setFinalConditions(Eigen::Ref<const Eigen::VectorXd> velocity,
                                     Eigen::Ref<const Eigen::VectorXd> /**acceleration*/)
{
    m_pimpl->finalCondition.velocity = velocity;
    m_pimpl->finalCondition.acceleration = Eigen::VectorXd::Zero(velocity.size());

    // The final conditions changed. The coefficients are outdated.
    m_pimpl->areCoefficientsComputed = false;

    if (m_pimpl->initialCondition.velocity.size() != 0 && !m_pimpl->knots.empty())
    {
        return this->evaluatePoint(m_pimpl->knots.front().timeInstant, m_pimpl->currentTrajectory);
    }

    return true;
}

bool CubicSpline::setAdvanceTimeStep(const double& dt)
{
    constexpr auto logPrefix = "[CubicSpline::setAdvanceTimeStep]";

    if (dt <= 0)
    {
        log()->error("{} The time step of the advance object has to be a strictly positive number.",
                     logPrefix);
        return false;
    }

    m_pimpl->advanceTimeStep = dt;

    return true;
}

bool CubicSpline::setKnots(const std::vector<Eigen::VectorXd>& position,
                           const std::vector<double>& time)
{
    constexpr auto logPrefix = "[CubicSpline::setKnots]";

    if (position.size() < 2)
    {
        log()->error("{} The number of knots has to be at least 2.", logPrefix);
        return false;
    }

    if (time.size() != position.size())
    {
        log()->error("{} The number of points has to be equal to the length of the time vector.",
                     logPrefix);
        return false;
    }

    // resize the vector containing all the knots
    m_pimpl->knots.resize(time.size());

    const std::size_t sizeOfVectors = position[0].size();
    for (std::size_t i = 0; i < m_pimpl->knots.size(); i++)
    {
        if (position[i].size() != sizeOfVectors)
        {
            log()->error("{} The size of the knot number {} is different from the one expected. "
                         "Expected: {} Retrieved {}.",
                         logPrefix,
                         i,
                         sizeOfVectors,
                         position[i].size());
            return false;
        }

        // set all the knots
        m_pimpl->resetKnot(time[i], position[i], m_pimpl->knots[i]);
    }

    // set the initial state for the advance interface
    m_pimpl->currentTrajectory.position = position.front();
    m_pimpl->advanceCurrentTime = time.front();

    // The knots changed. The coefficients are outdated.
    m_pimpl->areCoefficientsComputed = false;

    if (m_pimpl->initialCondition.velocity.size() != 0
        && m_pimpl->finalCondition.velocity.size() != 0)
    {
        return this->evaluatePoint(m_pimpl->knots.front().timeInstant, m_pimpl->currentTrajectory);
    }

    return true;
}

void CubicSpline::Impl::resetKnot(const double& timeInstant,
                                  Eigen::Ref<const Eigen::VectorXd> position,
                                  Knot& knot)
{
    knot.timeInstant = timeInstant;
    knot.position = position;

    const std::size_t sizeOfVectors = position.size();
    knot.velocity.resize(sizeOfVectors);
    knot.acceleration.resize(sizeOfVectors);
}

bool CubicSpline::Impl::computePhasesDuration()
{
    constexpr auto errorPrefix = "[CubicSpline::Impl::computePhasesDuration]";

    for (std::size_t i = 0; i < knots.size() - 1; i++)
    {
        polynomials[i].duration = knots[i + 1].timeInstant - knots[i].timeInstant;

        // This is required or stability purposes, the matrix A may not be invertible.
        if (std::abs(polynomials[i].duration) <= std::numeric_limits<double>::epsilon())
        {
            log()->error("{} Two consecutive points have the same time coordinate.", errorPrefix);
            return false;
        }
    }
    return true;
}

bool CubicSpline::Impl::computeCoefficients()
{
    constexpr auto errorPrefix = "[CubicSpline::Impl::computeCoefficients]";
    // check if the user already sets the knots
    if (knots.empty())
    {
        log()->error("{} Please set the knots before computing the coefficients.", errorPrefix);
        return false;
    }

    // the number of polynomials is equal to the "number of knots - 1"
    this->polynomials.resize(knots.size() - 1);

    // set the velocity and acceleration boundary conditions
    if (!this->setBoundaryVelocitiesAndAcceleration())
    {
        log()->error("{} Unable to set the boundary conditions related to the velocity and "
                     "acceleration.",
                     errorPrefix);
        return false;
    }

    // compute the duration of each phase (sub-trajectory)
    if (!this->computePhasesDuration())
    {
        log()->error("{} Unable to compute the phase duration.", errorPrefix);
        return false;
    }

    // populate the polynomials vector with the knots
    for (int i = 0; i < polynomials.size(); i++)
    {
        polynomials[i].initialPoint = &(knots[i]);
        polynomials[i].finalPoint = &(knots[i + 1]);
    }

    // intermediate conditions have to be computed only if the knots are greater than equal 3
    if (knots.size() > 2)
    {
        this->computeIntermediateVelocities();
    }

    // populate the coefficients of each polynomial
    for (auto& poly : polynomials)
    {
        this->setPolynomialCoefficients(poly);
    }

    // the coefficients have been computed
    areCoefficientsComputed = true;

    return true;
}

bool CubicSpline::Impl::setBoundaryVelocitiesAndAcceleration()
{
    constexpr auto errorPrefix = "[CubicSpline::Impl::setBoundaryVelocitiesAndAcceleration]";

    // this is an internal function and it is called only after setting the knots. However
    // the following assert checks that the number of knots is at least 1
    assert(!knots.empty());

    const int knotSize = knots.front().position.size();

    if (knotSize != initialCondition.velocity.size())
    {
        log()->error("{} The size of the initial velocity is different from the expected one. "
                     "Expected: {}. Retrieved: {}.",
                     errorPrefix,
                     knotSize,
                     initialCondition.velocity.size());
        return false;
    }

    if (knotSize != initialCondition.acceleration.size())
    {
        log()->error("{} The size of the initial acceleration is different from the expected one. "
                     "Expected: {}. Retrieved: {}.",
                     errorPrefix,
                     knotSize,
                     initialCondition.acceleration.size());
        return false;
    }

    if (knotSize != finalCondition.velocity.size())
    {
        log()->error("{} The size of the final velocity is different from the expected one. "
                     "Expected: {}. Retrieved: {}.",
                     errorPrefix,
                     knotSize,
                     finalCondition.velocity.size());
        return false;
    }

    if (knotSize != finalCondition.acceleration.size())
    {
        log()->error("{} The size of the final acceleration is different from the expected one. "
                     "Expected: {}. Retrieved: {}.",
                     errorPrefix,
                     knotSize,
                     finalCondition.acceleration.size());
        return false;
    }

    // store the boundary conditions
    knots.front().velocity = initialCondition.velocity;
    knots.front().acceleration = initialCondition.acceleration;

    knots.back().velocity = finalCondition.velocity;
    knots.back().acceleration = finalCondition.acceleration;

    return true;
}

void CubicSpline::Impl::addTripletCurrentKnot(const int& knotIndex,
                                              const int& rowOffset,
                                              const int& columnOffset,
                                              std::vector<Eigen::Triplet<double>>& tripletList)
{
    const auto& poly = polynomials[knotIndex];
    const auto& prevPoly = polynomials[knotIndex - 1];

    // The following triplet represent this matrix
    //  /      /   1      1\ \
    //  |   4  |------ - --| |
    //  |      |T        T | |
    //  |      \ i - 1    i/ |
    //  \                    /

    tripletList.emplace_back(rowOffset,
                             columnOffset,
                             4 * (1 / prevPoly.duration + 1 / poly.duration));
}

void CubicSpline::Impl::addTripletPreviousKnot(const int& knotIndex,
                                               const int& rowOffset,
                                               const int& columnOffset,
                                               std::vector<Eigen::Triplet<double>>& tripletList)
{
    const auto& poly = polynomials[knotIndex - 1];

    // The following triplet represent this matrix
    // /     2     \
    // |  -------   |
    // |  T         |
    // \   i - 1    /

    tripletList.emplace_back(rowOffset, columnOffset, 2 / poly.duration);
}

void CubicSpline::Impl::addTripletNextKnot(const int& knotIndex,
                                           const int& rowOffset,
                                           const int& columnOffset,
                                           std::vector<Eigen::Triplet<double>>& tripletList)
{
    const auto& poly = polynomials[knotIndex];

    // The following triplet represent this matrix
    // /    2    \
    // |   ---    |
    // |   T      |
    // \    i     /

    tripletList.emplace_back(rowOffset, columnOffset, 2 / poly.duration);
}

void CubicSpline::Impl::addKnownTermKnotPosition(const std::size_t& knotIndex,
                                                 const std::size_t& coordinateIndex,
                                                 Eigen::Ref<Eigen::VectorXd> b)
{
    const auto& poly = polynomials;

    const auto& i = knotIndex;
    const auto& j = coordinateIndex;

    b[0] += 6
            * ((knots[i].position[j] - knots[i - 1].position[j]) / std::pow(poly[i - 1].duration, 2)
               + (knots[i + 1].position[j] - knots[i].position[j]) / std::pow(poly[i].duration, 2));
}

void CubicSpline::Impl::addKnownTermNextKnot(const std::size_t& knotIndex,
                                             const std::size_t& coordinateIndex,
                                             Eigen::Ref<Eigen::VectorXd> b)
{
    const auto& poly = polynomials;
    const auto& i = knotIndex;
    const auto& j = coordinateIndex;
    b[0] -= 2 * knots[i + 1].velocity[j] / poly[i].duration;
}

void CubicSpline::Impl::addKnownTermPreviousKnot(const std::size_t& knotIndex,
                                                 const std::size_t& coordinateIndex,
                                                 Eigen::Ref<Eigen::VectorXd> b)
{
    const auto& poly = polynomials;

    const auto& i = knotIndex;
    const auto& j = coordinateIndex;

    b[0] -= 2 * knots[i - 1].velocity[j] / poly[i - 1].duration;
}

void CubicSpline::Impl::setPolynomialCoefficients(Polynomial& poly)
{
    const auto& T = poly.duration;

    const auto& x0 = poly.initialPoint->position;
    const auto& dx0 = poly.initialPoint->velocity;

    const auto& xT = poly.finalPoint->position;
    const auto& dxT = poly.finalPoint->velocity;

    poly.a0 = x0;
    poly.a1 = dx0;
    poly.a2 = -(3 * x0 - 3 * xT + 2 * T * dx0 + T * dxT) / (T * T);
    poly.a3 = (2 * x0 - 2 * xT + T * dx0 + T * dxT) / (T * T * T);
}

void CubicSpline::Impl::computeIntermediateVelocities()
{
    // here we assume that at least 3 points has been defined
    const std::size_t numberOfInteriorKnots = knots.size() - 2;

    Eigen::SparseMatrix<double> A(numberOfInteriorKnots, numberOfInteriorKnots);
    std::vector<Eigen::Triplet<double>> tripletsList;

    // Given a set of interior points the we can define a matrix A as
    //                          __                                        __
    //                         | x x 0 0 0 0 0 0 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //                         | x x x 0 0 0 0 0 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //                         | 0 x x x 0 0 0 0 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //     /\      ______      | 0 0 x x x 0 0 0 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //    /  \    |______|     | 0 0 0 x x x 0 0 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //   / /\ \    ______      |         ........................          |
    //  / ____ \  |______|     |         ........................          |
    // /_/    \_\              |         ........................          |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... x x x 0 0 0 0 |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... 0 x x x 0 0 0 |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... 0 0 x x x 0 0 |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... 0 0 0 x x x 0 |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... 0 0 0 0 x x x |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... 0 0 0 0 0 x x |
    //                         |__                                       __|
    //
    // where x represents a non zero number and 0 represents the number 0. The matrix A can be
    // stored as a sparse matrix where the number of non zero entries for the extremum elements are
    // 2 and the number of non zero elements for the interior (non extremum elements) are 3
    // The matrix A is a square matrix whose number of columns (and rows) is equal to the
    // number of interior knots of the spline times. If there is only an interior knot, A is a 1x1
    // dense matrix.
    unsigned int numberOfExpectedTriplets = 1;
    if (numberOfInteriorKnots > 1)
    {
        constexpr std::size_t numberOfExtremumElements = 2;
        constexpr std::size_t numberOfNonExtremumElements = 4;
        constexpr std::size_t numberOfExtremum = 2;
        numberOfExpectedTriplets = numberOfNonExtremumElements //
                                       * (numberOfInteriorKnots - numberOfExtremum)
                                   + numberOfExtremum * numberOfExtremumElements;
    }
    tripletsList.reserve(numberOfExpectedTriplets);

    for (int i = 0; i < numberOfInteriorKnots; i++)
    {
        const int absoluteKnotIndex = i + 1;

        // in this case there is only one interior knot A will be a 2
        if (i == 0 && (i + 1) == numberOfInteriorKnots)
        {
            this->addTripletCurrentKnot(absoluteKnotIndex, 0, 0, tripletsList);
        } else if (i == 0)
        {
            this->addTripletCurrentKnot(absoluteKnotIndex, i, 0, tripletsList);
            this->addTripletNextKnot(absoluteKnotIndex, i, 1, tripletsList);
        } else if (i + 1 == numberOfInteriorKnots)
        {
            this->addTripletPreviousKnot(absoluteKnotIndex, i, (i - 1), tripletsList);
            this->addTripletCurrentKnot(absoluteKnotIndex, i, (i - 1) + 1, tripletsList);
        } else
        {
            this->addTripletPreviousKnot(absoluteKnotIndex, i, (i - 1), tripletsList);
            this->addTripletCurrentKnot(absoluteKnotIndex, i, (i - 1) + 1, tripletsList);
            this->addTripletNextKnot(absoluteKnotIndex, i, (i - 1) + 1 + 1, tripletsList);
        }
    }

    A.setFromTriplets(tripletsList.begin(), tripletsList.end());

    // compute first coordinate
    Eigen::VectorXd b(A.rows());
    for (size_t j = 0; j < knots.front().position.size(); j++)
    {
        b.setZero();
        for (size_t i = 0; i < numberOfInteriorKnots; i++)
        {
            const int absoluteKnotIndex = i + 1;

            if (i == 0 && (i + 1) == numberOfInteriorKnots)
            {
                this->addKnownTermKnotPosition(absoluteKnotIndex, j, b.head<1>());
                this->addKnownTermNextKnot(absoluteKnotIndex, j, b.head<1>());
                this->addKnownTermPreviousKnot(absoluteKnotIndex, j, b.head<1>());

            } else if (i == 0)
            {
                this->addKnownTermKnotPosition(absoluteKnotIndex, j, b.head<1>());
                this->addKnownTermPreviousKnot(absoluteKnotIndex, j, b.head<1>());

            } else if (i + 1 == numberOfInteriorKnots)
            {
                this->addKnownTermNextKnot(absoluteKnotIndex, j, b.segment<1>(i));
                this->addKnownTermKnotPosition(absoluteKnotIndex, j, b.segment<1>(i));
            }

            else
            {
                this->addKnownTermKnotPosition(absoluteKnotIndex, j, b.segment<2>(i));
            }
        }

        Eigen::SparseQR<Eigen::SparseMatrix<double>,
                        Eigen::COLAMDOrdering<Eigen::SparseMatrix<double>::StorageIndex>>
            qrDecomposition;
        qrDecomposition.compute(A);
        Eigen::VectorXd solution = qrDecomposition.solve(b);

        for (size_t i = 0; i < numberOfInteriorKnots; i++)
        {
            const int absoluteKnotIndex = i + 1;

            knots[absoluteKnotIndex].velocity[j] = solution[i];
        }
    }
}

void CubicSpline::Impl::getPositionAtTime(const double& t,
                                            const Polynomial& poly,
                                            Eigen::Ref<Eigen::VectorXd> position)
{
    // improve the readability of the code
    const auto& a0 = poly.a0;
    const auto& a1 = poly.a1;
    const auto& a2 = poly.a2;
    const auto& a3 = poly.a3;

    position = a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3);
}

void CubicSpline::Impl::getVelocityAtTime(const double& t,
                                            const Polynomial& poly,
                                            Eigen::Ref<Eigen::VectorXd> velocity)
{
    // improve the readability of the code
    const auto& a1 = poly.a1;
    const auto& a2 = poly.a2;
    const auto& a3 = poly.a3;

    velocity = a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2);
}

void CubicSpline::Impl::getAccelerationAtTime(const double& t,
                                              const Polynomial& poly,
                                              Eigen::Ref<Eigen::VectorXd> acceleration)
{
    // improve the readability of the code
    const auto& a2 = poly.a2;
    const auto& a3 = poly.a3;

    acceleration = 2 * a2 + 2 * 3 * a3 * t;
}

bool CubicSpline::evaluatePoint(const double& t,
                                Eigen::Ref<Eigen::VectorXd> position,
                                Eigen::Ref<Eigen::VectorXd> velocity,
                                Eigen::Ref<Eigen::VectorXd> acceleration)
{
    constexpr auto logPrefix = "[CubicSpline::evaluatePoint]";
    if (!m_pimpl->areCoefficientsComputed)
    {
        if (!m_pimpl->computeCoefficients())
        {
            log()->error("{} Unable to compute the coefficients of the spline.", logPrefix);
            return false;
        }
    }

    if (t < m_pimpl->polynomials.front().initialPoint->timeInstant)
    {
        constexpr double initialTime = 0;

        m_pimpl->getPositionAtTime(initialTime, m_pimpl->polynomials.front(), position);
        m_pimpl->getVelocityAtTime(initialTime, m_pimpl->polynomials.front(), velocity);
        m_pimpl->getAccelerationAtTime(initialTime, m_pimpl->polynomials.front(), acceleration);


        return true;
    }

    if (t >= m_pimpl->polynomials.back().finalPoint->timeInstant)
    {
        const double finalTime = m_pimpl->polynomials.back().finalPoint->timeInstant
                                 - m_pimpl->polynomials.back().initialPoint->timeInstant;
        m_pimpl->getPositionAtTime(finalTime, m_pimpl->polynomials.back(), position);
        m_pimpl->getVelocityAtTime(finalTime, m_pimpl->polynomials.back(), velocity);
        m_pimpl->getAccelerationAtTime(finalTime, m_pimpl->polynomials.back(), acceleration);
        return true;
    }

    const auto poly = std::find_if(m_pimpl->polynomials.begin(),
                                   m_pimpl->polynomials.end(),
                                   [&t](const auto& p) {
                                       return ((t >= p.initialPoint->timeInstant)
                                               && (t < p.finalPoint->timeInstant));
                                   });

    if (poly == m_pimpl->polynomials.end())
    {
        log()->error("{} Unable to find the sub-trajectory.", logPrefix);
        return false;
    }

    m_pimpl->getPositionAtTime(t - poly->initialPoint->timeInstant, *poly, position);
    m_pimpl->getVelocityAtTime(t - poly->initialPoint->timeInstant, *poly, velocity);
    m_pimpl->getAccelerationAtTime(t - poly->initialPoint->timeInstant, *poly, acceleration);

    return true;
}

bool CubicSpline::evaluatePoint(const double& t, SplineState& state)
{
    return this->evaluatePoint(t, state.position, state.velocity, state.acceleration);
}

bool CubicSpline::isOutputValid() const
{
    // if the time step is different from zero and the user already set the knots the advance
    // capabilities can be used
    bool ok = (m_pimpl->advanceTimeStep != 0.0) && (!m_pimpl->knots.empty());
    return ok;
}

bool CubicSpline::advance()
{
    constexpr auto logPrefix = "[CubicSpline::advance]";
    if (!this->isOutputValid())
    {
        log()->error("{} The advance capabilities cannot ne used. Have you set the advance time "
                     "step?",
                     logPrefix);
        return false;
    }

    // advance the time step
    m_pimpl->advanceCurrentTime = std::min(m_pimpl->advanceTimeStep + m_pimpl->advanceCurrentTime,
                                           m_pimpl->knots.back().timeInstant);

    // Here we can improve the performances by avoiding calling the evaluatePoint function
    // indeed this function performs a binary search to find the sub-trajectory for a given time
    // instant. When the advance is called the sequence of the sub-trajectory is already
    // predetermined.
    return this->evaluatePoint(m_pimpl->advanceCurrentTime, m_pimpl->currentTrajectory);
}

const SplineState& CubicSpline::getOutput() const
{
    return m_pimpl->currentTrajectory;
}
