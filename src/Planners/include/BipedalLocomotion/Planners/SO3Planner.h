/**
 * @file SO3Planner.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_H

#include <manif/SO3.h>

namespace BipedalLocomotion
{
namespace Planners
{

/**
 * Representation used in the SO3Planner
 */
enum class Representation
{
    LeftTrivialized,
    RightTrivialized
};

/**
 * SO3Planner implements a minimum jerk trajectory planner for object belonging to SO(3). The
 * planner assumes initial and final angular acceleration and velocity of the object equal to zero.
 * The generated velocity and acceleration are written in the inertial or in body-fixed frame
 * accordingly to the chosen representation.
 * @tparam chosen representation:
 * - The right trivialization planner generates a velocity and an acceleration expressed in the
 * inertial frame.
 * - The left trivialization planner generates a velocity and an acceleration expressed in the
 * body-fixed frame.
 */
template <Representation representation> class SO3Planner
{
    /** Initial rotation from the inertial frame to the body frame. Namely
     * $\f{}^{\mathcal{I}}R_{\mathcal{B}}$\f */
    manif::SO3d m_initialRotation{manif::SO3d::Identity()};

    /** Distance bewteen the initial rotation and the final one. The definition of the distance
     * depends on the chosen Representation */
    manif::SO3d::Tangent m_distance{manif::SO3d::Tangent::Zero()};

    double m_T{1.0}; /**< Trajectory duration in seconds */

public:
    /**
     * Set the rotation
     * @param initialRotation initial rotation $\f{}^{\mathcal{I}}R_{\mathcal{B}_0}$\f.
     * @param finalRotation final rotation $\f{}^{\mathcal{I}}R_{\mathcal{B}_T}$\f.
     * @param duration Trajectory duration in seconds.
     * @return True in case of success/false otherwise.
     */
    bool setRotations(const manif::SO3d& initialRotation,
                      const manif::SO3d& finalRotation,
                      const double& duration);

    /**
     * Get the trajectory at a given time
     * @param time time, in seconds, at which the trajectory should be computed.
     * @param rotation $\f{}^{\mathcal{I}}R_{\mathcal{B}}$\f.
     * @param velocity angular velocity expressed in inertial or body fixed frame (accordingly to
     * the trivialization used).
     * @param acceleration angular acceleration expressed in inertial or body fixed frame
     * (accordingly to the trivialization used).
     * @return True in case of success/false otherwise.
     */
    template<class Derived>
    bool evaluatePoint(const double& time,
                       manif::SO3d& rotation,
                       manif::SO3TangentBase<Derived>& velocity,
                       manif::SO3TangentBase<Derived>& acceleration) const;
};

/**
 * The right trivialization planner generates a velocity and an acceleration expressed in the
 * inertial frame.
 */
using SO3PlannerInertial = SO3Planner<Representation::RightTrivialized>;

/**
 * The left trivialization planner generates a velocity and an acceleration expressed in the
 * body-fixed frame.
 */
using SO3PlannerBody = SO3Planner<Representation::LeftTrivialized>;

} // namespace Planners
} // namespace BipedalLocomotion

#include <BipedalLocomotion/Planners/SO3Planner.tpp>

#endif // BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_H
