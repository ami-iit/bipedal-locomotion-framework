/**
 * @file TimeVaryingDCMPlanner.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <casadi/casadi.hpp>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/CubicSpline.h>

#include <BipedalLocomotion/Planners/TimeVaryingDCMPlanner.h>
#include <BipedalLocomotion/Planners/ConvexHullHelper.h>

using namespace BipedalLocomotion::Planners;

/**
 * Private implementation of the TimeVaryingDCMPlanner class
 */
struct TimeVaryingDCMPlanner::Impl
{
    static constexpr std::size_t dcmVectorSize = 3;
    static constexpr std::size_t vrpVectorSize = 3;

    bool isTrajectoryComputed{false}; /**< True if the trajectory has been computed */
    bool isPlannerInitialized{false}; /**< True if the planner has been initialized */

    ConvexHullHelper convexHullHelper; /**< Convex hull helper. It is used to compute the
                                          constraints related to the position of the Zero Moment
                                          Point (ZMP) */

    iDynTree::CubicSpline dcmRefX; /**< Cubic spline for the x-coordinate of the reference DCM */
    iDynTree::CubicSpline dcmRefY; /**< Cubic spline for the y-coordinate of the reference DCM */
    iDynTree::CubicSpline dcmRefZ; /**< Cubic spline for the z-coordinate of the reference DCM */

    casadi::Opti opti; /**< CasADi opti stack */
    casadi::MX costFunction; /**< Cost function of the optimization problem */

    /**
     * OptimizationVariables contains the optimization variables expressed as CasADi elements.
     */
    struct OptimizationVariables
    {
        casadi::MX dcm;
        casadi::MX omega;
        casadi::MX vrp;
        casadi::MX omegaDot;
    };
    OptimizationVariables optiVariables; /**< Optimization variables */

    /**
     * OptimizationParameters contains some parameters used in the optimization problem. A parameter
     * is a constant value considered in the optimization problem. Before solving the problem the
     * parameters have to be filled with numerical values.
     */
    struct OptimizationParameters
    {
        casadi::MX dcmInitialPosition; /**< Initial boundary condition on the DCM position */
        casadi::MX dcmFinalPosition; /**< Final boundary condition on the DCM position */
        casadi::MX dcmInitialVelocity; /**< Initial boundary condition on the DCM velocity */
        casadi::MX dcmFinalVelocity; /**< Final boundary condition on the DCM velocity */
        casadi::MX omegaInitialValue; /**< Initial boundary condition on the omega */
        casadi::MX omegaFinalValue; /**< Final boundary condition on the omega */
        casadi::MX dcmRefererenceTraj; /** DCM trajectory. This is trajectory is used as
                                          regularization parameter while solving the optimization
                                          problem */
    };
    OptimizationParameters optiParameters; /**< Optimization parameters */

    /**
     * Utilities function used in the optimization problem
     */
    struct OptimizationFunctions
    {
        casadi::Function dcmDynamics; /** DCM dynamics */
        casadi::Function ecmp; /** Enhanced centroidal moment pivot (eCMP) equation */
    };
    OptimizationFunctions optiFunctions; /**< Collection of useful functions */

    /**
     * OptimizationSolution contains some elements required to retrieve the solution of the
     * optimization problem.
     */
    struct OptimizationSolution
    {
        std::unique_ptr<casadi::OptiSol> solution; /**< Pointer to the solution of the optimization
                                                      problem */
        casadi::DM dcm; /**< Optimal DCM trajectory */
        casadi::DM omega; /**< Optimal omega trajectory */
        casadi::DM vrp; /**< Optimal VRP trajectory */
        casadi::DM omegaDot; /**< Optimal omega rate of change trajectory */
    };
    OptimizationSolution optiSolution; /**< Solution of the optimization problem */

    DCMPlannerState trajectory; /**< State of the planner at the current state */
    std::size_t trajectoryIndex; /**< Current index of the trajectory */
    std::size_t numberOfTrajectorySamples; /**< Number of the samples considered by the optimization
                                              problem */

    /**
     * Setting of the optimization problem
     */
    struct OptimizationSettings
    {
        unsigned long solverVerbosity{1}; /**< Verbosity of ipopt */
        std::string ipoptLinearSolver{"ma27"}; /**< Linear solved used by ipopt */

        double plannerSamplingTime; /**< Sampling time of the planner in seconds */
        std::vector<iDynTree::Position> footCorners; /**< Position of the corner of the foot
                                                    expressed in a frame placed in the center of the
                                                    foot. The X axis points forward and the z
                                                    upword.*/

        double omegaDotWeight; /**< Weight related to the omega dot */
        double dcmTrackingWeight; /**< Weight to the tracking of the DCM */
        double omegaDotRateOfChangeWeight; /**< Weight related to rate of change of omega dot */
        double vrpRateOfChangeWeight; /**< Weight related to the rate of change of the VRP */
        double dcmRateOfChangeWeight; /**< Weight related to the rate of change of the DCM */
    };
    OptimizationSettings optiSettings; /**< Settings */

    /**
     * Get the DCM dynamics
     */
    casadi::Function getDCMDynamics()
    {
        casadi::MX dcm = casadi::MX::sym("dcm", 3);
        casadi::MX omega = casadi::MX::sym("omega");
        casadi::MX vrp = casadi::MX::sym("vrp", 3);
        casadi::MX omegaDot = casadi::MX::sym("omega_dot");

        casadi::MX rhs = casadi::MX::vertcat(
            {(omega - omegaDot / omega) * (dcm - vrp), omegaDot});

        return casadi::Function("DCMDynamics", {dcm, omega, vrp, omegaDot}, {rhs});
    }

    /**
     * Get the eCMP from VRP and omega
     */
    casadi::Function getECMP()
    {
        casadi::MX omega = casadi::MX::sym("omega");
        casadi::MX vrp = casadi::MX::sym("vrp", 3);
        casadi::MX omegaDot = casadi::MX::sym("omega_dot");

        casadi::MX rhs = casadi::MX::vertcat(
            {vrp(casadi::Slice(0, 2)), vrp(2) - 9.81 / (casadi::MX::pow(omega, 2) - omegaDot)});

        return casadi::Function("eCMP", {omega, vrp, omegaDot}, {rhs});
    }

    /**
     * Clear the optimization problem
     */
    void clear()
    {
        opti = casadi::Opti();
    }

    /**
     * Setup the optimization problem options
     */
    void setupOptiOptions()
    {
        casadi::Dict casadiOptions;
        casadi::Dict ipoptOptions;

        if (this->optiSettings.solverVerbosity != 0)
        {
            casadi_int ipoptVerbosity = static_cast<long long>(optiSettings.solverVerbosity - 1);
            ipoptOptions["print_level"] = ipoptVerbosity;
            casadiOptions["print_time"] = true;
        } else
        {
            ipoptOptions["print_level"] = 0;
            casadiOptions["print_time"] = false;
        }
        ipoptOptions["linear_solver"] = this->optiSettings.ipoptLinearSolver;

        this->opti.solver("ipopt", casadiOptions, ipoptOptions);
    }

    /**
     * Setup opti stack
     */
    void setupOpti(const std::size_t& horizonSampling)
    {
        using Sl = casadi::Slice;

        // define the size of the optimization variables
        this->optiVariables.dcm = this->opti.variable(this->dcmVectorSize, horizonSampling + 1);
        this->optiVariables.omega = this->opti.variable(1, horizonSampling + 1);
        this->optiVariables.vrp = this->opti.variable(this->vrpVectorSize, horizonSampling);
        this->optiVariables.omegaDot = this->opti.variable(1, horizonSampling);

        // set the initial constraints
        this->optiParameters.dcmInitialPosition = this->opti.parameter(this->dcmVectorSize);
        this->optiParameters.dcmFinalPosition = this->opti.parameter(this->dcmVectorSize);
        this->optiParameters.dcmInitialVelocity = this->opti.parameter(this->dcmVectorSize);
        this->optiParameters.dcmFinalVelocity = this->opti.parameter(this->dcmVectorSize);
        this->optiParameters.omegaInitialValue = this->opti.parameter();
        this->optiParameters.omegaFinalValue = this->opti.parameter();

        // set constraints and cost functions whose equations does not change in the optimization
        // problem
        auto& dcm = this->optiVariables.dcm;
        auto& vrp = this->optiVariables.vrp;
        auto& omega = this->optiVariables.omega;
        auto& omegaDot = this->optiVariables.omegaDot;

        // initial values
        this->opti.subject_to(omega(Sl(), 0) == this->optiParameters.omegaInitialValue);
        this->opti.subject_to(dcm(Sl(), 0) == this->optiParameters.dcmInitialPosition);
        this->opti.subject_to(casadi::MX::vertcat(this->optiFunctions.dcmDynamics(
                                  {dcm(Sl(), 0), omega(Sl(), 0), vrp(Sl(), 0), omegaDot(Sl(), 0)}))
                              == casadi::MX::vertcat({this->optiParameters.dcmInitialVelocity, 0}));

        // final values
        this->opti.subject_to(omega(Sl(), -1) == this->optiParameters.omegaFinalValue);
        this->opti.subject_to(dcm(Sl(), -1) == this->optiParameters.dcmFinalPosition);
        this->opti.subject_to(casadi::MX::vertcat(this->optiFunctions.dcmDynamics(
                {dcm(Sl(), -1), omega(Sl(), -1), vrp(Sl(), -1), omegaDot(Sl(), -1)}))
            == casadi::MX::vertcat({this->optiParameters.dcmFinalVelocity, 0}));

        for(std::size_t i = 0; i < horizonSampling - 1; i++)
        {
            // system dynamics
            this->opti.subject_to(
                casadi::MX::vertcat({dcm(Sl(), i + 1), omega(Sl(), i + 1)})
                == casadi::MX::vertcat({dcm(Sl(), i), omega(Sl(), i)})
                       + this->optiSettings.plannerSamplingTime
                             * casadi::MX::vertcat(this->optiFunctions.dcmDynamics(
                                 {dcm(Sl(), i), omega(Sl(), i), vrp(Sl(), i), omegaDot(Sl(), i)})));

            // omega has to be positive
            this->opti.subject_to(omega(Sl(), i) > 0);
            this->opti.subject_to(casadi::MX::pow(omega(Sl(), i), 2) - omegaDot(Sl(), i) > 0);
        }

        this->optiParameters.dcmRefererenceTraj = this->opti.parameter(this->dcmVectorSize, horizonSampling + 1);

        this->costFunction
            = this->optiSettings.omegaDotWeight * casadi::MX::sumsqr(this->optiVariables.omegaDot)
              + this->optiSettings.dcmTrackingWeight
                    * casadi::MX::sumsqr(this->optiVariables.dcm
                                         - this->optiParameters.dcmRefererenceTraj)
              + this->optiSettings.omegaDotRateOfChangeWeight
                    * casadi::MX::sumsqr(casadi::MX::diff(this->optiVariables.omegaDot))
              + this->optiSettings.vrpRateOfChangeWeight
                    * casadi::MX::sumsqr(casadi::MX::diff(this->optiVariables.vrp.T()))
              + this->optiSettings.dcmRateOfChangeWeight
                    * casadi::MX::sumsqr(casadi::MX::diff(this->optiVariables.dcm.T()));

        this->opti.minimize(costFunction);

        // set the opti object options
        this->setupOptiOptions();
    }

    /**
     * Get the CasADi functions
     */
    void setupCasADiFunctions()
    {
        optiFunctions.dcmDynamics = getDCMDynamics();
        optiFunctions.ecmp = getECMP();
    }

    bool computeConstraintElementsECMP(const ContactPhase& contactPhase,
                                       casadi::DM& ecmpConstraintA,
                                       casadi::DM& ecmpConstraintB)
    {
        // TODO we may optimize here (DYNAMIC MEMORY ALLOCATION)
        Eigen::MatrixXd points;

        // check if the points belongs to the same plane
        int numberOfCoordinates = 3;

        if (contactPhase.activeContacts.size() == 1)
        {
            numberOfCoordinates = 2;
        }
        else if (contactPhase.activeContacts.size() == 2)
        {
            auto contactIt = contactPhase.activeContacts.cbegin();
            double foot1Height = contactIt->second->pose.getPosition()[2];
            std::advance(contactIt, 1);
            double foot2Height = contactIt->second->pose.getPosition()[2];

            if (foot2Height == foot1Height)
            {
                numberOfCoordinates = 2;
            }
        }

        const std::size_t numberOfPoints
            = contactPhase.activeContacts.size() * this->optiSettings.footCorners.size();

        points.resize(numberOfCoordinates, numberOfPoints);

        // compute the convex hull only once
        int columnIndex = 0;
        iDynTree::Position point;
        for (const auto& [contactName, activeContact] : contactPhase.activeContacts)
        {
            for (const auto& footCorner : this->optiSettings.footCorners)
            {
                point = activeContact->pose * footCorner;
                points.col(columnIndex) = iDynTree::toEigen(point).head(numberOfCoordinates);
                columnIndex++;
            }
        }
        this->convexHullHelper.buildConvexHull(points);

        // TODO PLEASE OPTIMIZE ME
        // please check https://github.com/casadi/casadi/issues/2563 and
        // https://groups.google.com/forum/#!topic/casadi-users/npPcKItdLN8
        // Assumption: the matrices as stored as column-major
        const auto& A = this->convexHullHelper.getA();
        ecmpConstraintA.resize(A.rows(), A.cols());
        ecmpConstraintA = casadi::DM::zeros(A.rows(), A.cols());
        std::memcpy(ecmpConstraintA.ptr(),
                    A.data(),
                    sizeof(double) * A.rows() * A.cols());

        const auto& b = this->convexHullHelper.getB();
        ecmpConstraintB.resize(b.size(), 1);
        ecmpConstraintB = casadi::DM::zeros(b.size(), 1);
        std::memcpy(ecmpConstraintB.ptr(), b.data(), sizeof(double) * b.size());

        return numberOfCoordinates == 2;
    }

    bool setupOptimizationProblem(std::shared_ptr<const ContactPhaseList> contactPhaseList,
                                  const DCMPlannerState& initialState)
    {
        using Sl = casadi::Slice;

        // set the initial conditions
        this->opti.set_value(this->optiParameters.dcmInitialPosition,
                             casadi::DM(std::vector<double>(initialState.dcmPosition.data(),
                                                            initialState.dcmPosition.data()
                                                                + this->dcmVectorSize)));
        this->opti.set_value(this->optiParameters.dcmInitialVelocity,
                             casadi::DM(std::vector<double>(initialState.dcmVelocity.data(),
                                                            initialState.dcmVelocity.data()
                                                                + this->dcmVectorSize)));
        this->opti.set_value(this->optiParameters.omegaInitialValue, initialState.omega);


        double time = contactPhaseList->cbegin()->beginTime;

        std::vector<Eigen::Vector3d> dcmKnots;
        std::vector<double> timeKnots;

        // first point
        timeKnots.push_back(time);
        dcmKnots.push_back(initialState.dcmPosition);

        double averageDCMHeight = initialState.dcmPosition[2];

        casadi::DM ecmpConstraintA;
        casadi::DM ecmpConstraintB;

        const auto& dcm = this->optiVariables.dcm;
        const auto& vrp = this->optiVariables.vrp;
        const auto& omega = this->optiVariables.omega;
        const auto& omegaDot = this->optiVariables.omegaDot;


        auto contactPhaseListIt = contactPhaseList->cbegin();


        bool feetAreInSamePlane = computeConstraintElementsECMP(*contactPhaseListIt,
                                                                ecmpConstraintA,
                                                                ecmpConstraintB);

        for (std::size_t k = 0; k < this->optiVariables.vrp.columns(); k++)
        {
            // if the
            if(k * this->optiSettings.plannerSamplingTime > contactPhaseListIt->endTime)
            {
                std::advance(contactPhaseListIt, 1);
                feetAreInSamePlane = computeConstraintElementsECMP(*contactPhaseListIt,
                                                                   ecmpConstraintA,
                                                                   ecmpConstraintB);

                if (contactPhaseListIt->activeContacts.size() == 2
                    && contactPhaseListIt != contactPhaseList->begin()
                    && contactPhaseListIt != contactPhaseList->lastPhase())
                {
                    timeKnots.emplace_back(
                        (contactPhaseListIt->endTime + contactPhaseListIt->beginTime) / 2);

                    auto contactIt = contactPhaseListIt->activeContacts.cbegin();
                    iDynTree::Position p1 = contactIt->second->pose.getPosition();
                    std::advance(contactIt, 1);
                    iDynTree::Position p2 = contactIt->second->pose.getPosition();

                    Eigen::Vector3d desiredDCMPosition;
                    desiredDCMPosition = (iDynTree::toEigen(p1) + iDynTree::toEigen(p2)) / 2;
                    desiredDCMPosition(2) += averageDCMHeight;

                    dcmKnots.emplace_back(desiredDCMPosition);
                }

                // TODO please try to make it more versatile
                // read it as if the robot is in the last double support phase
                else if (contactPhaseListIt->activeContacts.size() == 2
                         && contactPhaseListIt == contactPhaseList->lastPhase())
                {
                    timeKnots.push_back(contactPhaseListIt->endTime);
                    auto contactIt = contactPhaseListIt->activeContacts.cbegin();
                    iDynTree::Position p1 = contactIt->second->pose.getPosition();
                    std::advance(contactIt, 1);
                    iDynTree::Position p2 = contactIt->second->pose.getPosition();

                    Eigen::Vector3d desiredDCMPosition;
                    desiredDCMPosition = (iDynTree::toEigen(p1) + iDynTree::toEigen(p2)) / 2;
                    desiredDCMPosition(2) += averageDCMHeight;

                    dcmKnots.emplace_back(desiredDCMPosition);
                }
            }

            if (!feetAreInSamePlane)
            {
                this->opti.subject_to(
                    casadi::MX::mtimes(ecmpConstraintA,
                                       casadi::MX::vertcat(this->optiFunctions.ecmp(
                                           {omega(Sl(), k), vrp(Sl(), k), omegaDot(Sl(), k)})))
                    <= ecmpConstraintB);
            } else
            {
                this->opti.subject_to(casadi::MX::mtimes(ecmpConstraintA, vrp(Sl(0, 2), k))
                                      <= ecmpConstraintB);

                this->opti.subject_to(
                    vrp(2, k) - 9.81 / (casadi::MX::pow(omega(Sl(), k), 2) - omegaDot(Sl(), k))
                    == contactPhaseListIt->activeContacts.begin()->second->pose.getPosition()[2]);
            }
        }

        dcmRefX.setInitialConditions(initialState.dcmVelocity[0], 0);
        dcmRefX.setFinalConditions(0, 0);

        dcmRefY.setInitialConditions(initialState.dcmVelocity[1], 0);
        dcmRefY.setFinalConditions(0, 0);

        dcmRefZ.setInitialConditions(initialState.dcmVelocity[2], 0);
        dcmRefZ.setFinalConditions(0, 0);

        iDynTree::VectorDynSize dcmKnotsX(dcmKnots.size());
        iDynTree::VectorDynSize dcmKnotsY(dcmKnots.size());
        iDynTree::VectorDynSize dcmKnotsZ(dcmKnots.size());
        iDynTree::VectorDynSize timeKnotsiDynTree(dcmKnots.size());

        for(int i = 0; i < dcmKnots.size(); i++)
        {
            dcmKnotsX[i] = dcmKnots[i][0];
            dcmKnotsY[i] = dcmKnots[i][1];
            dcmKnotsZ[i] = dcmKnots[i][2];
            timeKnotsiDynTree[i] = timeKnots[i];
        }


        // set last values
        this->opti.set_value(this->optiParameters.dcmFinalPosition,
                             casadi::DM(std::vector<double>(dcmKnots.back().data(),
                                                            dcmKnots.back().data()
                                                                + this->dcmVectorSize)));

        this->opti.set_value(this->optiParameters.dcmFinalVelocity,
                             casadi::DM::zeros(this->dcmVectorSize));

        // TODO the final omega is equal to the initial omega
        this->opti.set_value(this->optiParameters.omegaFinalValue, initialState.omega);

        dcmRefX.setData(timeKnotsiDynTree, dcmKnotsX);
        dcmRefY.setData(timeKnotsiDynTree, dcmKnotsY);
        dcmRefZ.setData(timeKnotsiDynTree, dcmKnotsZ);

        casadi::DM initialValueDCM(3,this->optiVariables.dcm.columns());
        casadi::DM initialValueOmega(1,this->optiVariables.dcm.columns());

        for (int i = 0; i < this->optiVariables.dcm.columns(); i++)
        {

            this->opti.set_value(this->optiParameters.dcmRefererenceTraj(0, i),
                                 dcmRefX.evaluatePoint(i * this->optiSettings.plannerSamplingTime));
            this->opti.set_value(this->optiParameters.dcmRefererenceTraj(1, i),
                                 dcmRefY.evaluatePoint(i * this->optiSettings.plannerSamplingTime));
            this->opti.set_value(this->optiParameters.dcmRefererenceTraj(2, i),
                                 dcmRefZ.evaluatePoint(i * this->optiSettings.plannerSamplingTime));

            initialValueDCM(0, i)
                = dcmRefX.evaluatePoint(i * this->optiSettings.plannerSamplingTime);
            initialValueDCM(1, i)
                = dcmRefY.evaluatePoint(i * this->optiSettings.plannerSamplingTime);
            initialValueDCM(2, i)
                = dcmRefZ.evaluatePoint(i * this->optiSettings.plannerSamplingTime);

            initialValueOmega(Sl(), i) = std::sqrt(
                9.81 / dcmRefZ.evaluatePoint(i * this->optiSettings.plannerSamplingTime));
        }

        this->opti.set_initial(this->optiVariables.dcm, initialValueDCM);
        this->opti.set_initial(this->optiVariables.omega, initialValueOmega);
        this->opti.set_initial(this->optiVariables.omegaDot,
                               casadi::DM::zeros(1, omegaDot.columns()));

        return true;
    }

    void prepareSolution()
    {
        std::size_t outputIndex = this->trajectoryIndex == numberOfTrajectorySamples
                                      ? numberOfTrajectorySamples - 1
                                      : trajectoryIndex;

        std::memcpy(this->trajectory.dcmPosition.data(),
                    optiSolution.dcm.ptr(),
                    sizeof(double) * this->dcmVectorSize);

        std::memcpy(this->trajectory.vrpPosition.data(),
                    optiSolution.vrp.ptr(),
                    sizeof(double) * this->dcmVectorSize);

        trajectory.omega = static_cast<double>(optiSolution.omega(0, trajectoryIndex));

        const double omegaDot = static_cast<double>(optiSolution.omegaDot(0, outputIndex));

        trajectory.dcmVelocity = (trajectory.omega - omegaDot / trajectory.omega)
                                 * (trajectory.dcmPosition - trajectory.vrpPosition);
    }
};

TimeVaryingDCMPlanner::TimeVaryingDCMPlanner()
{
    m_pimpl = std::make_unique<Impl>();
    assert(m_pimpl);
}

TimeVaryingDCMPlanner::~TimeVaryingDCMPlanner() = default;

bool TimeVaryingDCMPlanner::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    assert(m_pimpl);

    // convert the weak_ptr into a shared_ptr
    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        std::cerr << "[TimeVaryingDCMPlanner::initialize] The pointer to the parameter handler is "
                     "expired."
                  << std::endl;
        return false;
    }

    if (!ptr->getParameter("planner_sampling_time", m_pimpl->optiSettings.plannerSamplingTime))
    {
        std::cerr << "[TimeVaryingDCMPlanner::initialize] Unable to load the sampling time of the "
                     "planner."
                  << std::endl;
        return false;
    }

    int numberOfFootCorners;
    if (!ptr->getParameter("number_of_foot_corners", numberOfFootCorners))
    {
        std::cerr << "[TimeVaryingDCMPlanner::initialize] Unable to load the number of foot "
                     "corners."
                  << std::endl;
        return false;
    }

    m_pimpl->optiSettings.footCorners.resize(numberOfFootCorners);

    for(std::size_t i = 0; i < numberOfFootCorners; i++)
    {
        if (!ptr->getParameter("foot_corner_" + std::to_string(i),
                               m_pimpl->optiSettings.footCorners[i]))
        {
            std::cerr << "[TimeVaryingDCMPlanner::initialize] Unable to load get the foot corner "
                         "number: "
                      << i << ". Please provide the foot corners having the following names: ";
            for (std::size_t j = 0; j < numberOfFootCorners; j++)
            {
                std::cerr << "foot_corner_" + std::to_string(j) << " ";
            }
            std::cerr << "." << std::endl;

            return false;
        }
    }

    bool okCostFunctions = true;
    okCostFunctions &= ptr->getParameter("omega_dot_weight", m_pimpl->optiSettings.omegaDotWeight);
    okCostFunctions &= ptr->getParameter("dcm_tracking_weight", m_pimpl->optiSettings.dcmTrackingWeight);
    okCostFunctions &= ptr->getParameter("omega_dot_rate_of_change_weight",
                                         m_pimpl->optiSettings.omegaDotRateOfChangeWeight);
    okCostFunctions &= ptr->getParameter("vrp_rate_of_change_weight",
                                         m_pimpl->optiSettings.vrpRateOfChangeWeight);

    okCostFunctions &= ptr->getParameter("dcm_rate_of_change_weight",
                                         m_pimpl->optiSettings.dcmRateOfChangeWeight);

    if (!okCostFunctions)
    {
        std::cerr << "[TimeVaryingDCMPlanner::initialize] Unable to load weights of the cost "
                     "function"
                  << std::endl;
        return false;
    }

    // the casadi functions are initialized only once
    m_pimpl->setupCasADiFunctions();

    m_pimpl->isPlannerInitialized = true;
    return true;
}

bool TimeVaryingDCMPlanner::computeTrajectory()
{
    assert(m_pimpl);

    if(!m_pimpl->isPlannerInitialized)
    {
        std::cerr << "[TimeVaryingDCMPlanner::initialize] Please initialize the planner before "
                     "computing the trajectory."
                  << std::endl;
        return false;
    }

    // clear the solver and the solution computed
    m_pimpl->clear();

    const double& initialTrajectoryTime = m_contactPhaseList->cbegin()->beginTime;
    const double& endTrajectoryTime = m_contactPhaseList->lastPhase()->endTime;

    m_pimpl->numberOfTrajectorySamples = std::ceil((endTrajectoryTime - initialTrajectoryTime)
                                                   / m_pimpl->optiSettings.plannerSamplingTime);

    m_pimpl->setupOpti(m_pimpl->numberOfTrajectorySamples);

    if (!m_pimpl->setupOptimizationProblem(m_contactPhaseList, m_initialState))
    {
        std::cerr << "[TimeVaryingDCMPlanner::computeTrajectory] Unable to setup the optimization "
                     "problem."
                  << std::endl;
        return false;
    }

    try
    {
        m_pimpl->optiSolution.solution = std::make_unique<casadi::OptiSol>(m_pimpl->opti.solve());
    } catch (const std::exception& e)
    {
        std::cerr << "[TimeVaryingDCMPlanner::computeTrajectory] Unable to solve the optimization "
                     "problem. The following exception has been thrown by the solver"
                  << e.what() << "." << std::endl;
        return false;
    }

    m_pimpl->optiSolution.dcm = m_pimpl->optiSolution.solution->value(m_pimpl->optiVariables.dcm);
    m_pimpl->optiSolution.vrp = m_pimpl->optiSolution.solution->value(m_pimpl->optiVariables.vrp);
    m_pimpl->optiSolution.omega = m_pimpl->optiSolution.solution->value(m_pimpl->optiVariables.omega);
    m_pimpl->optiSolution.omegaDot = m_pimpl->optiSolution.solution->value(m_pimpl->optiVariables.omegaDot);

    m_pimpl->prepareSolution();

    m_pimpl->isTrajectoryComputed = true;

    return true;
}

const DCMPlannerState& TimeVaryingDCMPlanner::get() const
{
    assert(m_pimpl);
    return m_pimpl->trajectory;
}

bool TimeVaryingDCMPlanner::isValid() const
{
    assert(m_pimpl);
    return m_pimpl->isTrajectoryComputed;
}

bool TimeVaryingDCMPlanner::advance()
{
    if (!isValid())
    {
        std::cerr << "[TimeVaryingDCMPlanner::advance] The data are not valid it is not possible "
                     "to advance."
                  << std::endl;
        return false;
    }

    m_pimpl->trajectoryIndex = std::min(m_pimpl->trajectoryIndex + 1, m_pimpl->numberOfTrajectorySamples);

    m_pimpl->prepareSolution();

    return true;
}
