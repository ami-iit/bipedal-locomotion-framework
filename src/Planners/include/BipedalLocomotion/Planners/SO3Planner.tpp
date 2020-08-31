/**
 * @file SO3Planner.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_TPP
#define BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_TPP

#include <iostream>

#include <manif/SO3.h>

#include <BipedalLocomotion/Planners/SO3Planner.h>

namespace BipedalLocomotion
{
namespace Planners
{

template <Representation representation>
bool SO3Planner<representation>::setRotations(const manif::SO3d& initialRotation,
                                              const manif::SO3d& finalRotation,
                                              const double& T)
{
    if (T <= 0)
    {
        std::cerr << "[SO3Planner::setRotation] The trajectory duration must be a strictly "
                     "positive number."
                  << std::endl;
        return false;
    }

    m_initialRotation = initialRotation;
    m_T = T;

    if constexpr (representation == Representation::RightTrivialized)
    {
        m_distance = (finalRotation * initialRotation.inverse()).log();
    } else
    // Please read it as representation == Representation::LeftTrivialized
    {
        m_distance = (initialRotation.inverse() * finalRotation).log();
    }

    return true;
}

template <Representation representation>
bool SO3Planner<representation>::evaluatePoint(const double& time,
                                               manif::SO3d& rotation,
                                               manif::SO3d::Tangent& velocity,
                                               manif::SO3d::Tangent& acceleration) const
{
    if (time < 0 || time > m_T)
    {
        std ::cerr << "[SO3Planner::evaluatePoint] The time has to be a real positive number "
                      "between 0 and "
                   << m_T << "." << std::endl;
        return false;
    }

    const double& t = time;
    // Compute the displacement in the tangent space
    // - displacement_tangent = log(finalRotation * initialRotation.inverse()) * s(t) (in case of
    //                                                                                 Right
    //                                                                                 Trivialization)
    // - displacement_tangent = log(initialRotation.inverse() * finalRotation) * s(t) (in case of
    //                                                                                 Left
    //                                                                                 Trivialization)
    const double s = 10 * std::pow(t / m_T, 3)
                     - 15 * std::pow(t / m_T, 4)
                     + 6 * std::pow(t / m_T, 5);
    const manif::SO3d::Tangent displacementTangent = m_distance * s;

    if constexpr (representation == Representation::RightTrivialized)
    {
        // please read this as R(t) = exp(displacementTangent) * initialRotation
        rotation = displacementTangent + m_initialRotation;
    } else
    // Please read it as representation == Representation::LeftTrivialized
    {
        // please read this as R(t) = initialRotation * exp(displacementTangent)
        rotation = m_initialRotation + displacementTangent;
    }

    // compute velocity (it is expressed in body / inertial frame accordingly to the chosen representation)
    const double sDot = 10 * 3 * std::pow(t, 2) / std::pow(m_T, 3)
                        - 15 * 4 * std::pow(t, 3) / std::pow(m_T, 4)
                        + 6 * 5 * std::pow(t, 4) / std::pow(m_T, 5);
    velocity = m_distance * sDot;

    // compute velocity (it is expressed in body / inertial frame accordingly to the chosen representation)
    const double sDdot = 10 * 3 * 2 * t / std::pow(m_T, 3)
                         - 15 * 4 * 3 * std::pow(t, 2) / std::pow(m_T, 4)
                         + 6 * 5 * 4 * std::pow(t, 3) / std::pow(m_T, 5);
    acceleration = m_distance * sDdot;

    return true;
}

} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_TPP
