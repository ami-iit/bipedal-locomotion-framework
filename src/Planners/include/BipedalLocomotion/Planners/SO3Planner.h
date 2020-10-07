/**
 * @file SO3Planner.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_H

#include <manif/SO3.h>

#include <BipedalLocomotion/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace Planners
{

/**
 * Trivialization used in the SO3Planner
 */
enum class LieGroupTrivialization
{
    Left,
    Right
};

/**
 * SO3PlannerState contains the state of the planner. The velocity and the acceleration are
 * expressed in inertial/body frame depending on the Trivialization used by the planner.
 */
struct SO3PlannerState
{
    manif::SO3d rotation; /**< \f${}^{\mathcal{I}}R_{\mathcal{B}}\f$ */
    manif::SO3d::Tangent velocity; /**< Angular velocity expressed in inertial or body fixed frame
                                    * (accordingly to the trivialization used). */
    manif::SO3d::Tangent acceleration; /**< Angular acceleration expressed in inertial or body fixed
                                        * frame (accordingly to the trivialization used). */
};

/**
 * SO3Planner implements a minimum jerk trajectory planner for object belonging to SO(3). The
 * planner assumes initial and final angular acceleration and velocity of the object equal to zero.
 * The generated velocity and acceleration are written in the inertial or in body-fixed frame
 * accordingly to the chosen trivialization.
 * @tparam chosen trivialization:
 * - The right trivialization planner generates a velocity and an acceleration expressed in the
 * inertial frame.
 * - The left trivialization planner generates a velocity and an acceleration expressed in the
 * body-fixed frame.
 */
template <LieGroupTrivialization trivialization>
class SO3Planner : public System::Advanceable<SO3PlannerState>
{
    /** Initial rotation from the inertial frame to the body frame. Namely
     * \f${}^{\mathcal{I}}R_{\mathcal{B}}\f$ */
    manif::SO3d m_initialRotation{manif::SO3d::Identity()};

    /** Distance bewteen the initial rotation and the final one. The definition of the distance
     * depends on the chosen Trivialization */
    manif::SO3d::Tangent m_distance{manif::SO3d::Tangent::Zero()};

    double m_T{1.0}; /**< Trajectory duration in seconds */

    double m_advanceTimeStep{0.0}; /**< Time step of the advance interface. */
    double m_advanceCurrentTime{0.0}; /**< Current time of the advance object. */
    SO3PlannerState m_state; /**< Current state of the planner. It is used by the advance
                                capabilities. */

public:
    /**
     * Set the rotation
     * @param initialRotation initial rotation \f${}^{\mathcal{I}}R_{\mathcal{B}_0}\f$.
     * @param finalRotation final rotation \f${}^{\mathcal{I}}R_{\mathcal{B}_T}\f$.
     * @param duration Trajectory duration in seconds.
     * @return True in case of success/false otherwise.
     */
    bool setRotations(const manif::SO3d& initialRotation,
                      const manif::SO3d& finalRotation,
                      const double& duration);

    /**
     * Get the trajectory at a given time
     * @param time time, in seconds, at which the trajectory should be computed.
     * @param state state of the planner.
     * @return True in case of success/false otherwise.
     */
    bool evaluatePoint(const double& time,
                       SO3PlannerState& state) const;

    /**
     * Get the trajectory at a given time
     * @param time time, in seconds, at which the trajectory should be computed.
     * @param rotation \f${}^{\mathcal{I}}R_{\mathcal{B}}\f$.
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


    /**
     * Set the time step of the advance interface.
     * @warning if the the time step is not set the user cannot use the advance features.
     * @param dt the time step of the advance block.
     * @return True in case of success, false otherwise.
     */
    bool setAdvanceTimeStep(const double& dt);

    /**
     * Get the state of the system.
     * @warning if the the time step of the advance is not set the user cannot use the advance
     * features.
     * @return a const reference of the requested object.
     */
    const SO3PlannerState& get() const final;

    /**
     * Determines the validity of the object retrieved with get()
     * @warning if the the time step of the advance is not set the user cannot use the advance
     * features.
     * @return True if the object is valid, false otherwise.
     */
    bool isValid() const final;

    /**
     * Advance the internal state. This may change the value retrievable from get().
     * @warning if the the time step of the advance is not set the user cannot use the advance
     * features.
     * @return True if the advance is successfull.
     */
    bool advance() final;
};

/**
 * The right trivialization planner generates a velocity and an acceleration expressed in the
 * inertial frame.
 */
using SO3PlannerInertial = SO3Planner<LieGroupTrivialization::Right>;

/**
 * The left trivialization planner generates a velocity and an acceleration expressed in the
 * body-fixed frame.
 */
using SO3PlannerBody = SO3Planner<LieGroupTrivialization::Left>;

} // namespace Planners
} // namespace BipedalLocomotion

#include <BipedalLocomotion/Planners/SO3Planner.tpp>

#endif // BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_H
