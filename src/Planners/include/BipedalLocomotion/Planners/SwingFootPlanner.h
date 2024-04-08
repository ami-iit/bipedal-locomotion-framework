/**
 * @file SwingFootPlanner.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_SWING_FOOT_PLANNER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_SWING_FOOT_PLANNER_H

#include <chrono>
#include <memory>

#include <manif/manif.h>

#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Math/Spline.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Planners/SO3Planner.h>
#include <BipedalLocomotion/System/Source.h>

namespace BipedalLocomotion
{
namespace Planners
{

/**
 * SwingFootPlannerState describes the state of Swing foot planner.
 */
struct SwingFootPlannerState
{
    manif::SE3d transform; /**< Homogeneous transform between the link and the inertial frame */
    manif::SE3d::Tangent mixedVelocity; /**< 6D-velocity written in mixed representation */
    manif::SE3d::Tangent mixedAcceleration; /**< 6D-acceleration written in mixed representation */
    bool isInContact{true}; /** < If true the link is in contact with the environment */
    std::chrono::nanoseconds time; /**< Time associated to the planned trajectory */
};

/**
 * SwingFootPlanner implement a minimum jerk trajectory planner for the swing foot. The planner is
 * designed in SE(3) and we assume that initial 6d-acceleration and 6d-velocity of the  foot is
 * always equal to zero at take off. The trajectory of the foot will belong to the Geodesic.
 * @note The user can decide to use a constant step height for the swing foot planner. In this case
 * the user must provide the step height. If the user decides to use a variable step height, the
 * user must provide the swing foot duration knots and the step height knots. In this latter case,
 * the planner will compute a first order spline for the step height. This can be used in case of
 * variable walking velocity where we want to modify the step height according to the walking speed.
 */
class SwingFootPlanner : public System::Source<SwingFootPlannerState>
{
    SwingFootPlannerState m_state; /**< State of the planner */

    std::chrono::nanoseconds m_dT{std::chrono::nanoseconds::zero()}; /**< Sampling time of the
                                                                        planner */

    /** Current time of the planner in seconds */
    std::chrono::nanoseconds m_currentTrajectoryTime{std::chrono::nanoseconds::zero()};

    /** Starting time of the active SO3 trajectory seconds */
    std::chrono::nanoseconds m_staringTimeOfCurrentSO3Traj{std::chrono::nanoseconds::zero()};

    Contacts::ContactList m_contactList; /**< List of the contacts */
    Contacts::PlannedContact m_lastValidContact; /**< This contains the current contact if
                                                      the contact is active otherwise the last
                                                    contact before the current swing phase. */

    SO3PlannerInertial m_SO3Planner; /**< Trajectory planner in SO(3) */
    std::unique_ptr<Math::Spline<Eigen::Vector2d>> m_planarPlanner; /**< Trajectory planner for the
                                                                       x y coordinates of the foot
                                                                     */
    std::unique_ptr<Math::Spline<Eigen::Matrix<double, 1, 1>>> m_heightPlanner; /**< Trajectory
                                                                                   planner for the
                                                                                   z coordinate of
                                                                                   the foot */

    /** Interpolator for the  Height of the swing foot. Note that this value could not be the
     * maximum height of the foot. If m_footApexTime is set to 0.5 the stepHeight is the maximum of
     * the trajectory. */
    std::unique_ptr<Math::Spline<Eigen::Matrix<double, 1, 1>>> m_stepHeightInterpolator;
    Eigen::Matrix<double, 1, 1> m_stepHeight; /**< Variable representing the step height */
    double m_maxStepHeight{0.0}; /**< Maximum value of the step height */

    double m_footApexTime{0.5}; /**< Number between 0 and 1 representing the foot apex instant */
    double m_positionTolerance{1e-6}; /**< Position tolerance in \f$m\f$ */
    double m_orientationTolerance{1e-6}; /**< Orientation tolerance in \f$rad\f$ */

    double m_footLandingVelocity{0.0}; /**< Landing velocity in \f$m/s\f$ */
    double m_footLandingAcceleration{0.0}; /**< Landing acceleration in \f$m/s^2\f$ */

    double m_footTakeOffVelocity{0.0}; /**< Take off velocity in \f$m/s\f$ */
    double m_footTakeOffAcceleration{0.0}; /**< Take off acceleration in \f$m/s^2\f$ */

    bool m_isOutputValid{false}; /**< True if getOutput returns meaningful data */

    /**
     * Evaluate the SE3 trajectory of the swing foot.
     * @param state will contain the pose, velocity and acceleration (expressed in mixed
     * representation) of the swing foot computed at the current time instant.
     * @return True in case of success/false otherwise.
     * @note This method assumes that the trajectory has been already created with the method
     * SwingFootPlanner::createSE3Traj.
     */
    bool evaluateSE3Traj(SwingFootPlannerState& state);

    /**
     * Create a new SE3Trajectory considering the previous and next contact.
     * @param initialPose initial pose of the foot.
     * @param initialPlanarVelocity initial planar velocity of the foot.
     * @param initialPlanarAcceleration initial planar acceleration of the foot.
     * @param initialVerticalVelocity initial vertical velocity of the foot.
     * @param initialVerticalAcceleration initial vertical acceleration of the foot.
     * @param initialAngularVelocity initial angular velocity of the foot.
     * @param initialAngularAcceleration initial angular acceleration of the foot.
     * @return True in case of success/false otherwise.
     * @note This method assumes that the final planar and angular velocity and acceleration are
     * equal to zero, while the final vertical velocity and acceleration are equal to the one
     * provided by the user.
     */
    bool createSE3Traj(const manif::SE3d& initialPose,
                       Eigen::Ref<const Eigen::Vector2d> initialPlanarVelocity,
                       Eigen::Ref<const Eigen::Vector2d> initialPlanarAcceleration,
                       Eigen::Ref<const Eigen::Matrix<double, 1, 1>> initialVerticalVelocity,
                       Eigen::Ref<const Eigen::Matrix<double, 1, 1>> initialVerticalAcceleration,
                       const manif::SO3d::Tangent& initialAngularVelocity,
                       const manif::SO3d::Tangent& initialAngularAcceleration);

public:
    // clang-format off
    /**
     * Initialize the planner.
     * @param handler pointer to the parameter handler.
     * @note the following parameters are required by the class
     * |         Parameter Name       |   Type   |                                                             Description                                                                                    | Mandatory |
     * |:----------------------------:|:--------:|:----------------------------------------------------------------------------------------------------------------------------------------------------------:|:---------:|
     * |        `sampling_time`       | `double` |                                               Sampling time of the planner in seconds                                                                      |    Yes    |
     * |        `foot_apex_time`      | `double` |            Number between 0 and 1 representing the foot apex instant. If 0 the apex happens at take off if 1 at touch down                                 |    Yes    |
     * |    `foot_landing_velocity`   | `double` |                                               Landing vertical velocity (default value 0.0)                                                                |    No     |
     * |  `foot_landing_acceleration` | `double` |                                             Landing vertical acceleration (default value 0.0)                                                              |    No     |
     * |   `foot_take_off_velocity`   | `double` |                                         Take-off vertical velocity (default value 0.0)                                                                     |    No     |
     * | `foot_take_off_acceleration` | `double` |                                       Take-off vertical acceleration (default value 0.0)                                                                   |    No     |
     * |    `interpolation_method`    | `string` | Define the interpolation method for the trajectory of the position. Accepted parameters: `min_acceleration`, `min_jerk` (default value `min_acceleration`) |    No     |
     * |    `position_tolerance`      | `double` |                          Position tolerance in meters considered in SwingFootPlanner::setContactList. (default value 1e-6)                                 |    No     |
     * |    `orientation_tolerance`   | `double` |                        Orientation tolerance in radians considered in SwingFootPlanner::setContactList. (default value 1e-6)                               |    No     |
     * |   `use_constant_step_height` |  `bool`  |                       Use a constant step height for the swing foot planner. If set to true `step_height` must be provided. If `false`, `step_height_time_knots` and `step_height_knots` need to be provided. (default value `true`)        |
     * |         `step_height`        | `double` |                              Height of the  swing foot. It is not the maximum height of the foot. If apex time is 0.5 `step_height` is the maximum         |  (see `use_constant_step_height`) |
     * |   `step_height_time_knots`   | `vector` |  Vector representing the swing foot duration time knots. Each element will be associated to the corresponding element of `step_height_knots`.              | (see `use_constant_step_height`) |
     * |      `step_height_knots`     | `vector` |  Vector the step height knots. Each element will be associated to the corresponding element of `step_height_time_knots`.                                   | (see `use_constant_step_height`) |
     * |       `max_step_height`      | `double` |    This is a safety parameter. Independently from the usage of different step height interpolation method, it sets the maximum value of step knot.         |    Yes     |
     * @return True in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) final;
    // clang-format on

    /**
     * Set the contact list
     * @param contactList contains the list for a given contact.
     * @return true in case of success, false otherwise.
     * @note The contact list can be updated at run-time, i.e., when the planner is running. However
     * the new contact list must satisfy a set of hypothesis.
     * If the contact list stored in the class is empty, then it is the first time the
     * contact list is added to the planner. In this case we accept all kinds of ContactList
     * If the contact list is not empty, we check if it is possible to update the list.  Given some
     * limitations of the framework (mainly due to the SO3 trajectory generation) for the time
     * being, we support only the two following cases:
     * - Given the current time instant, both the stored and the new contact lists must have an
     * active contact at the same pose. In details the check is done by comparing the position and
     * orientation of the contact. The position and orientation are considered equal if the
     * following two conditions are satisfied:
     *   1. the distance between the two positions is less than position_tolerance
     *   2. the distance between the two orientations is less than orientation_tolerance
     * - If the contact is not active (swing phase) the next contact must satisfy the following two
     * hypothesis
     *   1. the final orientation may change still the error (in the tangent space) between the
     *      new orientation and the current one should be parallel to the current velocity and
     *      acceleration vectors. This is required to keep the SO3Planner problem still treatable
     *      online. This check is not done here since the SO3Planner will complain in case of
     *      issues.
     *   2. the impact time of the next contact must be the same as the one of the next contact in
     *      the contact list stored in the class.
     */
    bool setContactList(const Contacts::ContactList& contactList);

    /**
     * Reset the time.
     * @param time internal time of the system.
     */
    void setTime(const std::chrono::nanoseconds& time);

    /**
     * @brief Get the object.
     * @return a const reference of the requested object.
     */
    const SwingFootPlannerState& getOutput() const final;

    /**
     * @brief Determines the validity of the object retrieved with get()
     * @return True if the object is valid, false otherwise.
     */
    bool isOutputValid() const final;

    /**
     * @brief Advance the internal state. This may change the value retrievable from get().
     * @return True if the advance is successfull.
     */
    bool advance() final;
};

} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_SWING_FOOT_PLANNER_H
