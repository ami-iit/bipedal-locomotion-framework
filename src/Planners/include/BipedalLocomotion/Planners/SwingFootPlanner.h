/**
 * @file SwingFootPlanner.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_SWING_FOOT_PLANNER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_SWING_FOOT_PLANNER_H

#include <memory>

#include <manif/manif.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Planners/QuinticSpline.h>
#include <BipedalLocomotion/Planners/SO3Planner.h>
#include <BipedalLocomotion/System/Advanceable.h>

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
};

/**
 * SwingFootPlanner implement a minimum jerk trajectory planner for the swing foot. The planner is
 * designed in SE(3) and we assume that initial 6d-acceleration and 6d-velocity of the  foot is
 * always equal to zero at take off. The trajectory of the foot will belong to the Geodesic.
 */
class SwingFootPlanner : public System::Advanceable<SwingFootPlannerState>
{
    SwingFootPlannerState m_state; /**< State of the planner */

    double m_dT{0.0}; /**< Sampling time of the planner in seconds*/
    double m_currentTrajectoryTime{0.0}; /**< Current time of the planner in seconds */

    Contacts::ContactList m_contactList; /**< List of the contacts */
    Contacts::ContactList::const_iterator m_currentContactPtr; /**< Pointer to the current contact. (internal
                                                        use) */

    SO3PlannerInertial m_SO3Planner; /**< Trajectory planner in SO(3) */
    QuinticSpline m_planarPlanner; /**< Trajectory planner for the x y coordinates of the foot */
    QuinticSpline m_heightPlanner; /**< Trajectory planner for the z coordinate of the foot */

    double m_stepHeight{0.0}; /**< Height of the swing foot. Note that this value could not be the
                                 maximum height of the foot. If m_footApexTime is set to 0.5 the
                                 stepHeight is the maximum of the trajectory. */
    double m_footApexTime{0.5}; /**< Number between 0 and 1 representing the foot apex instant */

    /**
     * Update the SE3 Trajectory.
     * @return True in case of success/false otherwise.
     */
    bool updateSE3Traj();

public:
    /**
     * Initialize the planner.
     * @param handler pointer to the parameter handler.
     * @note the following parameters are required by the class
     * |        Parameter Name       |   Type   |                                                      Description                                                     | Mandatory |
     * |:---------------------------:|:--------:|:--------------------------------------------------------------------------------------------------------------------:|:---------:|
     * |       `sampling_time`       | `double` |                                        Sampling time of the planner in seconds                                       |    Yes    |
     * |        `step_height`        | `double` | Height of the swing foot. It is not the maximum height of the foot. If apex time is 0.5 `step_height` is the maximum |    Yes    |
     * |       `foot_apex_time`      | `double` |   Number between 0 and 1 representing the foot apex instant. If 0 the apex happens at take off if 1 at touch down    |    Yes    |
     * |   `foot_landing_velocity`   | `double` |                                               Landing vertical velocity                                              |    Yes    |
     * | `foot_landing_acceleration` | `double` |                                             Landing vertical acceleration                                            |    Yes    |
     * @return True in case of success/false otherwise.
     */
    bool initialize(std::shared_ptr<ParametersHandler::IParametersHandler> handler);

    /**
     * Set the contact list
     * @param contactList contains the list fora given contact
     */
    void setContactList(const Contacts::ContactList& contactList);

    /**
     * @brief Get the object.
     * @return a const reference of the requested object.
     */
     const SwingFootPlannerState& get() const final;

    /**
     * @brief Determines the validity of the object retrieved with get()
     * @return True if the object is valid, false otherwise.
     */
     bool isValid() const final;

    /**
     * @brief Advance the internal state. This may change the value retrievable from get().
     * @return True if the advance is successfull.
     */
     bool advance() final;
};

} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_SWING_FOOT_PLANNER_H
