/**
 * @file SO3Planner.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_TPP
#define BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_TPP

#include <chrono>
#include <iostream>

#include <manif/SO3.h>

#include <BipedalLocomotion/Planners/SO3Planner.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
namespace Planners
{

template <class Rep, class Period>
constexpr double durationToSeconds(const std::chrono::duration<Rep, Period>& d)
{
    return std::chrono::duration<double>(d).count();
}

template <LieGroupTrivialization trivialization>
bool SO3Planner<trivialization>::computeCoefficients()
{
    constexpr double tolerance = 1e-3;
    constexpr auto logPrefix = "[SO3Planner::computeCoefficients]";

    // no need to compute the coefficients again
    if (areCoefficientsComputed)
    {
        return true;
    }

    if (!this->initialCondition.isSet)
    {
        log()->error("{} The initial boundary condition is not set.", logPrefix);
        return false;
    }

    if (!this->finalCondition.isSet)
    {
        log()->error("{} The final boundary condition is not set.", logPrefix);
        return false;
    }

    // in the following function we check that vector anc rotationError are linear dependent and we
    // compute the coefficient such that "vector = coefficient * rotationError".
    auto findVectorCoefficient = [tolerance, logPrefix](const manif::SO3d::Tangent& rotationError,
                                                        const manif::SO3d::Tangent& vector,
                                                        double& coefficient) -> bool {
        if (rotationError.coeffs().isZero())
        {
            coefficient = 0;
            return true;
        }

        if (vector.coeffs().isZero())
        {
            coefficient = 0;
            return true;
        }

        const double dotProduct = rotationError.coeffs().dot(vector.coeffs());
        const double rotationErrorNorm = rotationError.coeffs().norm();
        const double vectorNorm = vector.coeffs().norm();

        const double cosOfAngleBetweenTwoVectors = dotProduct / (rotationErrorNorm * vectorNorm);
        constexpr double cosOfTwoParallelVectors = 1;
        // the cosine of the angle of two parallel vector can be 1 or -1 for this reason we compute
        // the absolute value
        if (std::abs(std::abs(cosOfAngleBetweenTwoVectors) - cosOfTwoParallelVectors) > tolerance)
        {
            return false;
        }

        for (int i = 0; i < rotationError.coeffs().size(); i++)
        {
            const double tmp = vector.coeffs()[i] / rotationError.coeffs()[i];
            if (!std::isinf(tmp) && !std::isnan(tmp))
            {
                coefficient = tmp;
                return true;
            }
        }

        return false;
    };

    // check if the boundary conditions are linear independent with the error between the two
    // rotation
    double kv0, kv1, ka0, ka1;
    if (!findVectorCoefficient(m_distance, initialCondition.velocity, kv0)
        || !findVectorCoefficient(m_distance, initialCondition.acceleration, ka0)
        || !findVectorCoefficient(m_distance, finalCondition.velocity, kv1)
        || !findVectorCoefficient(m_distance, finalCondition.acceleration, ka1))
    {
        log()->error("{} The velocity and the boundary acceleration must be a linear dependent to "
                     "the vector representing the axis between the two rotations.",
                     logPrefix);
        return false;
    }

    const double T = durationToSeconds(m_T);

    polynomial.a0 = 0;
    polynomial.a1 = kv0;
    polynomial.a2 = ka0 / 2.0;
    polynomial.a3 = -(12 * T * kv0 + 8 * T * kv1 + 3 * T * T * ka0 - T * T * ka1 - 20) //
                    / (2 * T * T * T);
    polynomial.a4 = (16 * T * kv0 + 14 * T * kv1 + 3 * T * T * ka0 - 2 * T * T * ka1 - 30)
                    / (2 * T * T * T * T);
    polynomial.a5 = -(6 * T * kv0 + 6 * T * kv1 + T * T * ka0 - T * T * ka1 - 12) //
                    / (2 * T * T * T * T * T);

    areCoefficientsComputed = true;

    return true;
}

template <LieGroupTrivialization trivialization>
bool SO3Planner<trivialization>::setRotations(const manif::SO3d& initialRotation,
                                              const manif::SO3d& finalRotation,
                                              const std::chrono::nanoseconds& duration)
{
    constexpr auto logPrefix = "[SO3Planner::setRotation]";

    if (duration <= std::chrono::nanoseconds::zero())
    {
        log()->error("{} The trajectory duration must be a strictly positive number.", logPrefix);
        return false;
    }

    m_initialRotation = initialRotation;
    m_T = duration;
    m_distance = this->computeDistance(initialRotation, finalRotation);

    // reset the advance current time
    m_advanceCurrentTime = std::chrono::nanoseconds::zero();

    // reset the state
    m_state.rotation = m_initialRotation;
    m_state.velocity.setZero();
    m_state.acceleration.setZero();
    this->areCoefficientsComputed = false;

    return true;
}

template <LieGroupTrivialization trivialization>
bool SO3Planner<trivialization>::setInitialConditions(const manif::SO3d::Tangent& velocity,
                                                      const manif::SO3d::Tangent& acceleration)
{
    this->initialCondition.velocity = velocity;
    this->initialCondition.acceleration = acceleration;
    this->initialCondition.isSet = true;
    this->areCoefficientsComputed = false;
    return true;
}

template <LieGroupTrivialization trivialization>
bool SO3Planner<trivialization>::setFinalConditions(const manif::SO3d::Tangent& velocity,
                                                    const manif::SO3d::Tangent& acceleration)
{
    this->finalCondition.velocity = velocity;
    this->finalCondition.acceleration = acceleration;
    this->finalCondition.isSet = true;
    this->areCoefficientsComputed = false;
    return true;
}

template <LieGroupTrivialization trivialization>
template <class Derived>
bool SO3Planner<trivialization>::evaluatePoint(const std::chrono::nanoseconds& time,
                                               manif::SO3d& rotation,
                                               manif::SO3TangentBase<Derived>& velocity,
                                               manif::SO3TangentBase<Derived>& acceleration)
{
    constexpr auto logPrefix = "[SO3Planner::evaluatePoint]";

    if (time < std::chrono::nanoseconds::zero() || time > m_T)
    {
        log()->error("{} The time has to be a real positive number between 0 and {}.",
                     logPrefix,
                     m_T);
        return false;
    }

    if (!this->computeCoefficients())
    {
        log()->error("{} Unable to compute the coefficients.", logPrefix);
        return false;
    }

    const double t = durationToSeconds(time);
    // Compute the displacement in the tangent space
    // - displacement_tangent = log(finalRotation * initialRotation.inverse()) * s(t) (in case of
    //                                                                                 Right
    //                                                                                 Trivialization)
    // - displacement_tangent = log(initialRotation.inverse() * finalRotation) * s(t) (in case of
    //                                                                                 Left
    //                                                                                 Trivialization)
    // You can find the computation of s in "Modern Robotics: Mechanics, Planning, and Control"
    // (Chapter 9.2)
    const double t2 = t * t;
    const double t3 = t * t2;
    const double t4 = t * t3;
    const double t5 = t * t4;
    const double s = polynomial.a0 + //
                     polynomial.a1 * t + //
                     polynomial.a2 * t2 + //
                     polynomial.a3 * t3 + //
                     polynomial.a4 * t4 + //
                     polynomial.a5 * t5;
    const manif::SO3d::Tangent displacementTangent = m_distance * s;

    if constexpr (trivialization == LieGroupTrivialization::Right)
    {
        // please read this as R(t) = exp(displacementTangent) * initialRotation
        rotation = displacementTangent.exp() * m_initialRotation;
    } else
    // Please read it as trivialization == LieGroupTrivialization::Left
    {
        // please read this as R(t) = initialRotation * exp(displacementTangent)
        rotation = m_initialRotation * displacementTangent.exp();
    }

    // compute velocity (it is expressed in body / inertial frame accordingly to the chosen
    // trivialization)
    // You can find the computation of dds in "Modern Robotics: Mechanics, Planning, and Control"
    // (Chapter 9.2)
    const double ds = polynomial.a1 + //
                      2 * polynomial.a2 * t + //
                      3 * polynomial.a3 * t2 + //
                      4 * polynomial.a4 * t3 + //
                      5 * polynomial.a5 * t4;
    velocity = m_distance * ds;

    // compute acceleration (it is expressed in body / inertial frame accordingly to the chosen
    // trivialization)
    // You can find the computation of dds in "Modern Robotics: Mechanics,
    // Planning, and Control" (Chapter 9.2)
    const double dds = 2 * polynomial.a2 + //
                       3 * 2 * polynomial.a3 * t + //
                       4 * 3 * polynomial.a4 * t2 + //
                       5 * 4 * polynomial.a5 * t3;
    acceleration = m_distance * dds;

    return true;
}

template <LieGroupTrivialization trivialization>
bool SO3Planner<trivialization>::evaluatePoint(const std::chrono::nanoseconds& time,
                                               SO3PlannerState& state)
{
    return this->evaluatePoint(time, state.rotation, state.velocity, state.acceleration);
}

template <LieGroupTrivialization trivialization>
bool SO3Planner<trivialization>::setAdvanceTimeStep(const std::chrono::nanoseconds& dt)
{
    constexpr auto logPrefix = "[SO3Planner::setAdvanceTimeStep]";
    if (dt <= std::chrono::nanoseconds::zero())
    {
        log()->error("{} The time step of the advance time has to be a strictly positive number.",
                     logPrefix);
        return false;
    }

    m_advanceTimeStep = dt;

    return true;
}

template <LieGroupTrivialization trivialization>
bool SO3Planner<trivialization>::isOutputValid() const
{
    // if the time step is different from zero
    return (m_advanceTimeStep != std::chrono::nanoseconds::zero());
}

template <LieGroupTrivialization trivialization> bool SO3Planner<trivialization>::advance()
{
    constexpr auto logPrefix = "[SO3Planner::advance]";
    if (!this->isOutputValid())
    {
        log()->error("{} The advance capabilities cannot be used. Did you set the advance time "
                     "step?",
                     logPrefix);
        return false;
    }

    // advance the time step
    m_advanceCurrentTime = std::min(m_advanceTimeStep + m_advanceCurrentTime, m_T);

    // update the state of the system
    return evaluatePoint(m_advanceCurrentTime, m_state);
}

template <LieGroupTrivialization trivialization>
const SO3PlannerState& SO3Planner<trivialization>::getOutput() const
{
    return m_state;
}

template <LieGroupTrivialization trivialization>
manif::SO3d::Tangent SO3Planner<trivialization>::computeDistance(const manif::SO3d& initialRotation,
                                                                 const manif::SO3d& finalRotation)
{
    if constexpr (trivialization == LieGroupTrivialization::Right)
    {
        return (finalRotation * initialRotation.inverse()).log();
    } else
    // Please read it as trivialization == LieGroupTrivialization::Left
    {
        return (initialRotation.inverse() * finalRotation).log();
    }
}

template <LieGroupTrivialization trivialization>
manif::SO3d::Tangent
SO3Planner<trivialization>::projectTangentVector(const manif::SO3d& initialRotation,
                                                 const manif::SO3d& finalRotation,
                                                 const manif::SO3d::Tangent& vector)
{
    const manif::SO3d::Tangent distanceVector = computeDistance(initialRotation, finalRotation);
    const double distance = distanceVector.coeffs().norm();
    if (distance < 1e-4)
    {
        return manif::SO3d::Tangent::Zero();
    }

    // here we project vector on the vector named distance
    manif::SO3d::Tangent ret;
    ret.coeffs() =  distanceVector.coeffs().dot(vector.coeffs()) / distance * distanceVector.coeffs();
    return ret;
}

} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_TPP
