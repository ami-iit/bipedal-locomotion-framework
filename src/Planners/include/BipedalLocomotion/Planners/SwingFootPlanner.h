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
#include <BipedalLocomotion/Planners/ContactList.h>
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
    manif::SE3d::Tangent spatialVelocity; /**< Spatial velocity written in mixed representation */
    manif::SE3d::Tangent spatialAcceleration; /**< Spatial acceleration written in mixed
                                                 representation */
    bool isInContact{true}; /** < If true the link is in contact with the environment */
};

/**
 * SwingFootPlanner implement a minimum jerk trajectory planner for the swing foot. The planner is
 * designed in SE(3) and we assume that initial and final spatial acceleration and velocity of the
 * foot is always equal to zero at take off and landing. The trajectory of the foot will belong to
 * the Geodesic.
 */
class SwingFootPlanner : public System::Advanceable<SwingFootPlannerState>
{
    SwingFootPlannerState m_state; /**< State of the planner */

    double m_dT{0.0}; /**< Sampling time of the planner in seconds*/
    double m_currentTrajectoryTime{0.0}; /**< Current time of the planner in seconds */

    ContactList m_contactList; /**< List of the contacts */
    ContactList::const_iterator m_currentContactPtr; /**< Pointer to the current contact. (internal
                                                        use) */

    SO3PlannerInertial m_SO3Planner;
    QuinticSpline m_planarPlanner;
    QuinticSpline m_heightPlanner;

    double m_stepHeight{0};
    double m_footApexTime{0};

    bool updateSE3Traj();

public:
    /**
     * Initialize the planner.
     * @param handler pointer to the parameter handler.
     * @return True in case of success/false otherwise.
     */
    bool initialize(std::shared_ptr<ParametersHandler::IParametersHandler> handler);

    /**
     * Set the contact list
     * @param contactList contains the list fora given contact
     */
    void setContactList(const ContactList& contactList);

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
