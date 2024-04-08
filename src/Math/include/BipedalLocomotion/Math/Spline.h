/**
 * @file Spline.h
 * @authors Giulio Romualdi
 * @copyright 20203 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_MATH_SPLINE_H
#define BIPEDAL_LOCOMOTION_MATH_SPLINE_H

#include <chrono>
#include <vector>

#include <Eigen/Dense>

#include <BipedalLocomotion/System/Source.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
namespace Math
{

/**
 * TrajectoryPoint is a struct that contains the information of a trajectory point.
 */
template <typename T> struct TrajectoryPoint
{
    T position; /**< Position of the spline. */
    T velocity; /**< Velocity (1st order derivative) of the spline. */
    T acceleration; /**< Acceleration (2nd order derivative) of the spline. */

    std::chrono::nanoseconds time; /**< Time instant of the spline. */
};

/**
 * Spline implement a basic spline.
 * @note The spline is defined as a set of polynomials. The coefficients of the polynomials are
 * computed using the boundary conditions. The boundary conditions are the initial and final
 * velocity and acceleration.
 * @tparam T type of the spline. T must be an Eigen type (i.e. Eigen::Vector3d).
 */
template <typename T> class Spline : public System::Source<TrajectoryPoint<T>>
{

    static_assert(is_base_of_template<Eigen::MatrixBase, T>::value,
                  "T must be an Eigen type (e.g. Eigen::Vector3d)");

public:
    /**
     * BoundaryConditions is a struct that contains the boundary conditions of the spline.
     */
    struct DerivativeBoundaryConditions
    {
        T velocity; /**< Velocity (1st order derivative) of the spline. */
        T acceleration; /**< Acceleration (2nd order derivative) of the spline. */
    };

    /**
     * Destructor.
     */
    virtual ~Spline() = default;

    /**
     * Set the time step of the advance interface.
     * @warning if the the time step is not set, the user cannot use the advance features.
     * @param dt the time step of the advance block.
     * @return True in case of success, false otherwise.
     */
    bool setAdvanceTimeStep(const std::chrono::nanoseconds& dt);

    /**
     * Set the knots of the spline.
     * @param position position of the knots in \f$\mathbb{R}^n\f$.
     * @param time vector containing the time instant of the knots.
     * @return True in case of success, false otherwise.
     */
    bool setKnots(const std::vector<T>& position, //
                  const std::vector<std::chrono::nanoseconds>& time);

    /**
     * Set the initial condition of the spline
     * @param initialConditions initial conditions of the spline.
     * @return True in case of success, false otherwise.
     */
    bool setInitialConditions(const DerivativeBoundaryConditions& initialConditions);

    /**
     * Set the final condition of the spline
     * @param finalConditions final conditions of the spline.
     * @return True in case of success, false otherwise.
     */
    bool setFinalConditions(const DerivativeBoundaryConditions& finalConditions);

    /**
     * Set the initial condition of the spline
     * @param velocity initial velocity (i.e. first derivative).
     * @param acceleration initial acceleration (i.e. second derivative).
     * @return True in case of success, false otherwise.
     */
    bool setInitialConditions(Eigen::Ref<const T> initialVelocity,
                              Eigen::Ref<const T> initialAcceleration);

    /**
     * Set the final condition of the spline
     * @param velocity final velocity (i.e. first derivative).
     * @param acceleration final acceleration (i.e. second derivative).
     * @return True in case of success, false otherwise.
     */
    bool setFinalConditions(Eigen::Ref<const T> finalVelocity, //
                            Eigen::Ref<const T> finalAcceleration);

    /**
     * Evaluate the spline at a given point
     * @param t instant time
     * @param position position at time t
     * @param velocity velocity at time t
     * @param acceleration acceleration at time t
     * @return True in case of success, false otherwise.
     */
    bool evaluatePoint(const std::chrono::nanoseconds& t, //
                       Eigen::Ref<T> position,
                       Eigen::Ref<T> velocity,
                       Eigen::Ref<T> acceleration);

    /**
     * Evaluate the spline at a given point
     * @param t instant time
     * @param position position at time t
     * @return True in case of success, false otherwise.
     */
    bool evaluatePoint(const std::chrono::nanoseconds& t, //
                       Eigen::Ref<T> position);

    /**
     * Evaluate the spline at a given point
     * @param t instant time
     * @param position position at time t
     * @param velocity velocity at time t
     * @return True in case of success, false otherwise.
     */
    bool evaluatePoint(const std::chrono::nanoseconds& t, //
                       Eigen::Ref<T> position,
                       Eigen::Ref<T> velocity);

    /**
     * Evaluate the spline at given set of points
     * @param t ordered vector containing the time instant
     * @param position position at time t
     * @param velocity velocity at time t
     * @param acceleration acceleration at time t
     * @return True in case of success, false otherwise.
     * @note The time vector has to be ordered. No check is performed.
     */
    bool evaluateOrderedPoints(const std::vector<std::chrono::nanoseconds>& t, //
                               std::vector<T>& position,
                               std::vector<T>& velocity,
                               std::vector<T>& acceleration);

    /**
     * Evaluate the spline at a given point
     * @param t instant time
     * @param state of the system
     * @return True in case of success, false otherwise.
     */
    bool evaluatePoint(const std::chrono::nanoseconds& t, TrajectoryPoint<T>& state);

    /**
     * Check if the output is valid.
     * @return True if the output is valid, false otherwise.
     */
    bool isOutputValid() const final;

    /**
     * Get the output of the system.
     * @return The output of the system.
     */
    const TrajectoryPoint<T>& getOutput() const final;

    /**
     * Advance the system.
     * @return True in case of success, false otherwise.
     * @note The user has to set the time step of the advance interface.
     */
    bool advance() final;

protected:
    /**
     * Polynomial is a struct that contains the information of a polynomial.
     */
    struct Polynomial
    {
        std::chrono::nanoseconds duration; /**< Duration of the polynomial. */
        TrajectoryPoint<T>* initialPoint; /**< Initial point of the polynomial. */
        TrajectoryPoint<T>* finalPoint; /**< Final point of the polynomial. */
        std::vector<T> coefficients; /**< Coefficients of the polynomial. */

        /**
         * Get the position (value) of the polynomial.
         * @param t instant time.
         * @param position position of the polynomial.
         */
        void getPosition(const std::chrono::nanoseconds& t, Eigen::Ref<T> position) const;

        /**
         * Get the velocity (1st derivative) of the polynomial.
         * @param t instant time.
         * @param velocity velocity of the polynomial.
         */
        void getVelocity(const std::chrono::nanoseconds& t, Eigen::Ref<T> velocity) const;

        /**
         * Get the acceleration (2nd derivative) of the polynomial.
         * @param t instant time.
         * @param acceleration acceleration of the polynomial.
         */
        void getAcceleration(const std::chrono::nanoseconds& t, Eigen::Ref<T> acceleration) const;
    };

    std::vector<TrajectoryPoint<T>> m_knots; /**< Knots of the spline. */
    std::vector<Polynomial> m_polynomials; /**< Polynomials of the spline. */

    /**
     * Compute the duration of each phase (sub-trajectory).
     * @return True in case of success, false otherwise.
     */
    bool computePhasesDuration();

    /**
     * Compute the intermediate quantities of the spline.
     * @return True in case of success, false otherwise.
     */
    virtual void computeIntermediateQuantities() = 0;

    /**
     * Set the coefficients of the polynomial.
     * @param polynomial polynomial to be updated.
     * @return True in case of success, false otherwise.
     */
    virtual void updatePolynomialCoefficients(Polynomial& polynomial) = 0;

private:
    T m_dummy; /**< Dummy variable used to avoid the creation of a temporary variable. */
    TrajectoryPoint<T> m_currentTrajectoryPoint; /**< Current trajectory. */
    bool m_isOutputValid{false}; /**< True if the output is valid. */

    std::chrono::nanoseconds m_advanceCurrentTime; /**< Current time of the advance interface. */
    bool m_areCoefficientsComputed{false}; /**< True if the coefficients are computed. */

    DerivativeBoundaryConditions m_initialConditions; /**< Initial conditions of the spline. */
    DerivativeBoundaryConditions m_finalConditions; /**< Final conditions of the spline. */

    std::chrono::nanoseconds m_dT; /**< Time step of the advance interface. */

    /**
     * Compute the coefficients of the spline. If the coefficients are already computed the function
     * is a no-op.
     * @note This function calls computeIntermediateQuantities and updatePolynomialCoefficients.
     * @return True in case of success, false otherwise.
     */
    bool computeCoefficients();

    /**
     * Set the boundary conditions related to the velocity and acceleration.
     * @return True in case of success, false otherwise.
     */
    bool setBoundaryVelocitiesAndAcceleration();
};

template <typename T>
void Spline<T>::Polynomial::getPosition(const std::chrono::nanoseconds& t,
                                        Eigen::Ref<T> position) const
{
    const double tS = std::chrono::duration<double>(t).count();
    position = coefficients[0];
    for (std::size_t i = 1; i < coefficients.size(); i++)
    {
        position += coefficients[i] * std::pow(tS, i);
    }
}

template <typename T>
void Spline<T>::Polynomial::getVelocity(const std::chrono::nanoseconds& t,
                                        Eigen::Ref<T> velocity) const
{
    // if this is a zero order polynomial the velocity is always zero
    if (coefficients.size() < 2)
    {
        // coefficients.size() is always greater than 0
        velocity = coefficients[0] * 0;
        return;
    }

    const double tS = std::chrono::duration<double>(t).count();
    velocity = coefficients[1];
    for (std::size_t i = 2; i < coefficients.size(); i++)
    {
        velocity += coefficients[i] * i * std::pow(tS, i - 1);
    }
}

template <typename T>
void Spline<T>::Polynomial::getAcceleration(const std::chrono::nanoseconds& t,
                                            Eigen::Ref<T> acceleration) const
{
    // if this is a zero or first order polynomial the acceleration is always zero
    if (coefficients.size() < 3)
    {
        // coefficients.size() is always greater than 0
        acceleration = coefficients[0] * 0;
        return;
    }

    const double tS = std::chrono::duration<double>(t).count();
    acceleration = 2 * coefficients[2];
    for (std::size_t i = 3; i < coefficients.size(); i++)
    {
        acceleration += coefficients[i] * i * (i - 1) * std::pow(tS, i - 2);
    }
}

template <typename T>
bool Spline<T>::setKnots(const std::vector<T>& knots,
                         const std::vector<std::chrono::nanoseconds>& time)
{
    constexpr auto logPrefix = "[Spline::setKnots]";

    auto resetKnot
        = [](const std::chrono::nanoseconds& time, const T& position, TrajectoryPoint<T>& knot) {
              knot.position = position;
              knot.time = time;
              const std::size_t sizeOfVectors = position.size();
              knot.velocity.resize(sizeOfVectors);
              knot.acceleration.resize(sizeOfVectors);
          };

    if (knots.size() < 2)
    {
        log()->error("{} The number of knots has to be at least 2.", logPrefix);
        return false;
    }

    if (time.size() != knots.size())
    {
        log()->error("{} The number of points has to be equal to the length of the time vector.",
                     logPrefix);
        return false;
    }

    // resize the vector containing all the knots
    m_knots.resize(time.size());

    const std::size_t sizeOfVectors = knots.begin()->size();
    int i = 0;
    for (const auto& knot : knots)
    {
        if (knot.size() != sizeOfVectors)
        {
            log()->error("{} The size of the knot is different from the one expected. "
                         "Expected: {} Retrieved {}.",
                         logPrefix,
                         sizeOfVectors,
                         knot.size());
            return false;
        }

        // set all the knots
        resetKnot(time[i], knot, m_knots[i]);
        i += 1;
    }

    // set the initial state for the advance interface
    m_currentTrajectoryPoint.position = m_knots.front().position;
    m_currentTrajectoryPoint.velocity = m_knots.front().velocity;
    m_currentTrajectoryPoint.acceleration = m_knots.front().acceleration;

    m_advanceCurrentTime = m_knots.front().time;

    // The knots changed. The coefficients are outdated.
    m_areCoefficientsComputed = false;

    // resize the dummy variable
    m_dummy.resize(sizeOfVectors);

    return true;
}

template <typename T>
bool Spline<T>::setInitialConditions(Eigen::Ref<const T> initialVelocity,
                                     Eigen::Ref<const T> initialAcceleration)
{
    return this->setInitialConditions({initialVelocity, initialAcceleration});
}

template <typename T>
bool Spline<T>::setInitialConditions(const DerivativeBoundaryConditions& initialConditions)
{
    if (initialConditions.velocity.size() != initialConditions.acceleration.size())
    {
        log()->error("[Spline::setInitialConditions] The size of the velocity is different from "
                     "the one of the acceleration. Expected: {} Retrieved: {}.",
                     initialConditions.velocity.size(),
                     initialConditions.acceleration.size());
        return false;
    }

    m_areCoefficientsComputed = false;

    m_initialConditions = initialConditions;
    return true;
}

template <typename T>
bool Spline<T>::setFinalConditions(Eigen::Ref<const T> finalVelocity,
                                   Eigen::Ref<const T> finalAcceleration)
{
    return this->setFinalConditions({finalVelocity, finalAcceleration});
}

template <typename T>
bool Spline<T>::setFinalConditions(const DerivativeBoundaryConditions& finalConditions)
{
    if (finalConditions.velocity.size() != finalConditions.acceleration.size())
    {
        log()->error("[Spline::setFinalConditions] The size of the velocity is different from "
                     "the one of the acceleration. Expected: {} Retrieved: {}.",
                     finalConditions.velocity.size(),
                     finalConditions.acceleration.size());
        return false;
    }

    m_areCoefficientsComputed = false;

    m_finalConditions = finalConditions;
    return true;
}

template <typename T> bool Spline<T>::computePhasesDuration()
{
    constexpr auto errorPrefix = "[Splines::computePhasesDuration]";

    for (std::size_t i = 0; i < m_knots.size() - 1; i++)
    {
        m_polynomials[i].duration = m_knots[i + 1].time - m_knots[i].time;

        // This is required or stability purposes.
        if (m_polynomials[i].duration == std::chrono::nanoseconds::zero())
        {
            log()->error("{} Two consecutive points have the same time coordinate.", errorPrefix);
            return false;
        }
    }
    return true;
}

template <typename T> bool Spline<T>::setBoundaryVelocitiesAndAcceleration()
{
    constexpr auto errorPrefix = "[Spline::setBoundaryVelocitiesAndAcceleration]";

    // this is an internal function and it is called only after setting the knots. However
    // the following assert checks that the number of knots is at least 1
    assert(!m_knots.empty());

    const int knotSize = m_knots.front().position.size();

    if (knotSize != m_initialConditions.velocity.size())
    {
        log()->error("{} The size of the initial velocity is different from the expected one. "
                     "Expected: {}. Retrieved: {}.",
                     errorPrefix,
                     knotSize,
                     m_initialConditions.velocity.size());
        return false;
    }

    if (knotSize != m_initialConditions.acceleration.size())
    {
        log()->error("{} The size of the initial acceleration is different from the expected one. "
                     "Expected: {}. Retrieved: {}.",
                     errorPrefix,
                     knotSize,
                     m_initialConditions.acceleration.size());
        return false;
    }

    if (knotSize != m_finalConditions.velocity.size())
    {
        log()->error("{} The size of the final velocity is different from the expected one. "
                     "Expected: {}. Retrieved: {}.",
                     errorPrefix,
                     knotSize,
                     m_finalConditions.velocity.size());
        return false;
    }

    if (knotSize != m_finalConditions.acceleration.size())
    {
        log()->error("{} The size of the final acceleration is different from the expected one. "
                     "Expected: {}. Retrieved: {}.",
                     errorPrefix,
                     knotSize,
                     m_finalConditions.acceleration.size());
        return false;
    }

    // store the boundary conditions
    m_knots.front().velocity = m_initialConditions.velocity;
    m_knots.front().acceleration = m_initialConditions.acceleration;

    m_knots.back().velocity = m_finalConditions.velocity;
    m_knots.back().acceleration = m_finalConditions.acceleration;

    return true;
}

template <typename T> bool Spline<T>::computeCoefficients()
{
    constexpr auto errorPrefix = "[Spline::computeCoefficients]";

    // check if the coefficients are already computed. In this case there is no need to recompute
    // them
    if (m_areCoefficientsComputed)
    {
        return true;
    }

    // check if the user already sets the knots
    if (m_knots.empty())
    {
        log()->error("{} Please set the knots before computing the coefficients.", errorPrefix);
        return false;
    }

    // the number of polynomials is equal to the "number of knots - 1"
    m_polynomials.resize(m_knots.size() - 1);

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
    for (std::size_t i = 0; i < m_polynomials.size(); i++)
    {
        m_polynomials[i].initialPoint = &(m_knots[i]);
        m_polynomials[i].finalPoint = &(m_knots[i + 1]);
    }

    // intermediate conditions have to be computed only if the knots are greater than equal 3
    // this function depends on the spline implementation
    if (m_knots.size() > 2)
    {
        this->computeIntermediateQuantities();
    }

    // populate the coefficients of each polynomial
    for (auto& poly : m_polynomials)
    {
        this->updatePolynomialCoefficients(poly);
    }

    // the coefficients have been computed
    m_areCoefficientsComputed = true;

    return true;
}

template <typename T>
bool Spline<T>::evaluatePoint(const std::chrono::nanoseconds& t, Eigen::Ref<T> position)
{
    return this->evaluatePoint(t, position, m_dummy, m_dummy);
}

template <typename T>
bool Spline<T>::evaluatePoint(const std::chrono::nanoseconds& t,
                              Eigen::Ref<T> position,
                              Eigen::Ref<T> velocity)
{
    return this->evaluatePoint(t, position, velocity, m_dummy);
}

template <typename T>
bool Spline<T>::evaluatePoint(const std::chrono::nanoseconds& t,
                              Eigen::Ref<T> position,
                              Eigen::Ref<T> velocity,
                              Eigen::Ref<T> acceleration)
{
    constexpr auto logPrefix = "[Spline::evaluatePoint]";

    if (!this->computeCoefficients())
    {
        log()->error("{} Unable to compute the coefficients of the spline.", logPrefix);
        return false;
    }

    if (t < m_polynomials.front().initialPoint->time)
    {
        constexpr std::chrono::nanoseconds initialTime = std::chrono::nanoseconds::zero();
        m_polynomials.front().getPosition(initialTime, position);
        m_polynomials.front().getVelocity(initialTime, velocity);
        m_polynomials.front().getAcceleration(initialTime, acceleration);
        return true;
    }

    if (t >= m_polynomials.back().finalPoint->time)
    {
        const auto finalTime = m_polynomials.back().finalPoint->time //
                               - m_polynomials.back().initialPoint->time;
        m_polynomials.back().getPosition(finalTime, position);
        m_polynomials.back().getVelocity(finalTime, velocity);
        m_polynomials.back().getAcceleration(finalTime, acceleration);
        return true;
    }

    const auto poly = std::find_if(m_polynomials.begin(), m_polynomials.end(), [&t](const auto& p) {
        return ((t >= p.initialPoint->time) && (t < p.finalPoint->time));
    });

    if (poly == m_polynomials.end())
    {
        log()->error("{} Unable to find the sub-trajectory.", logPrefix);
        return false;
    }

    poly->getPosition(t - poly->initialPoint->time, position);
    poly->getVelocity(t - poly->initialPoint->time, velocity);
    poly->getAcceleration(t - poly->initialPoint->time, acceleration);

    return true;
}

template <typename T>
bool Spline<T>::evaluateOrderedPoints(const std::vector<std::chrono::nanoseconds>& t,
                                      std::vector<T>& position,
                                      std::vector<T>& velocity,
                                      std::vector<T>& acceleration)
{
    constexpr auto logPrefix = "[Spline::evaluateOrderedPoints]";

    if (!this->computeCoefficients())
    {
        log()->error("{} Unable to compute the coefficients of the spline.", logPrefix);
        return false;
    }

    // resize the output vectors
    position.resize(t.size());
    velocity.resize(t.size());
    acceleration.resize(t.size());

    auto poly = m_polynomials.begin();

    for (std::size_t i = 0; i < t.size(); i++)
    {
        // check if the time instant is before the first knot
        if (t[i] < m_polynomials.front().initialPoint->time)
        {
            constexpr std::chrono::nanoseconds initialTime = std::chrono::nanoseconds::zero();
            poly->getPosition(initialTime, position[i]);
            poly->getVelocity(initialTime, velocity[i]);
            poly->getAcceleration(initialTime, acceleration[i]);
            continue;
        }

        // check if the time instant is after the last knot
        if (t[i] >= m_polynomials.back().finalPoint->time)
        {
            const auto finalTime = m_polynomials.back().finalPoint->time //
                                   - m_polynomials.back().initialPoint->time;
            poly->getPosition(finalTime, position[i]);
            poly->getVelocity(finalTime, velocity[i]);
            poly->getAcceleration(finalTime, acceleration[i]);
            continue;
        }

        if (poly != m_polynomials.end() && t[i] >= poly->finalPoint->time)
        {
            // advance the iterator since the time instant is greater than the final time of the
            // polynomial function. This is an optimization since the time instants are ordered.
            poly = std::find_if(std::next(poly, 1),
                                m_polynomials.end(),
                                [&t, &i](const auto& p) -> bool {
                                    return ((t[i] >= p.initialPoint->time)
                                            && (t[i] < p.finalPoint->time));
                                });
        }

        if (poly == m_polynomials.end())
        {
            log()->error("{} Unable to find the sub-trajectory.", logPrefix);
            return false;
        }

        poly->getPosition(t[i] - poly->initialPoint->time, position[i]);
        poly->getVelocity(t[i] - poly->initialPoint->time, velocity[i]);
        poly->getAcceleration(t[i] - poly->initialPoint->time, acceleration[i]);
    }

    return true;
}

template <typename T>
bool Spline<T>::evaluatePoint(const std::chrono::nanoseconds& t, TrajectoryPoint<T>& state)
{
    state.time = t;
    return this->evaluatePoint(t, state.position, state.velocity, state.acceleration);
}

template <typename T> bool Spline<T>::isOutputValid() const
{
    return m_isOutputValid;
}

template <typename T> const TrajectoryPoint<T>& Spline<T>::getOutput() const
{
    return m_currentTrajectoryPoint;
}

template <typename T> bool Spline<T>::advance()
{
    m_isOutputValid = false;

    // Here we can improve the performances by avoiding calling the evaluatePoint function
    // indeed this function performs a binary search to find the sub-trajectory for a given time
    // instant. When the advance is called the sequence of the sub-trajectory is already
    // predetermined.
    if (!this->evaluatePoint(m_advanceCurrentTime, m_currentTrajectoryPoint))
    {
        log()->error("[Spline::advance] Unable to evaluate the spline at the current time.");
        return false;
    }

    // advance the time step
    m_advanceCurrentTime = std::min(m_dT + m_advanceCurrentTime, m_knots.back().time);

    m_isOutputValid = true;

    return true;
}

template <typename T> bool Spline<T>::setAdvanceTimeStep(const std::chrono::nanoseconds& dt)
{
    if (dt <= std::chrono::nanoseconds::zero())
    {
        log()->error("[Spline::setAdvanceTimeStep] The time step has to be greater than zero.");
        return false;
    }

    m_dT = dt;
    return true;
}

} // namespace Math
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATH_SPLINE_H
