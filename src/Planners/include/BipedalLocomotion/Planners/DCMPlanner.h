/**
 * @file DCMPlanner.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_DCM_PLANNER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_DCM_PLANNER_H

#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/System/Source.h>

namespace BipedalLocomotion
{
namespace Planners
{

/**
 * DCMPlannerState describes the state of the Divergent Component of Motion (DCM) planner.
 */
struct DCMPlannerState
{
    Eigen::Vector3d dcmPosition; /**< Position of the DCM expressed w.r.t. the inertial frame */
    Eigen::Vector3d dcmVelocity; /**< Velocity of the DCM expressed w.r.t. the inertial frame */
    Eigen::Vector3d vrpPosition; /**< Position of the virtual repellent point (VRP) expressed
                                    w.r.t. the inertial frame */
    double omega;/**< Value of the parameter omega */
    double omegaDot;/**< Value of the parameter omega dot */
};

/**
 * DCMPlanner defines a trajectory generator for the variable height Divergent
 * component of motion (DCM). Please inherit publicly from this class in order to define your
 * planner.
 */
class DCMPlanner : public BipedalLocomotion::System::Source<DCMPlannerState>
{
protected:
    Contacts::ContactPhaseList m_contactPhaseList; /**< Contact phases. */

    DCMPlannerState m_initialState; /**< Initial state of the planner */

public:
    /**
     * Set the initial state of the planner
     * @param initialState the initial state of the planner
     */
    virtual void setInitialState(const DCMPlannerState& initialState);

    /**
     * Set the contact phase list
     * @note the contactPhaseList pointer should point to an already initialized ContactPhaseList.
     * Please be sure that the memory pointed is reachable for the entire life time of the
     * DCMPlanner class.
     * @param contactPhaseList pointer containing the list of the contact phases
     * @return true in case of success, false otherwise.
     */
    virtual bool setContactPhaseList(const Contacts::ContactPhaseList& contactPhaseList);

    /**
     * Compute the DCM trajectory.
     * @warning Please implement the function in your custom planner.
     * @return true in case of success, false otherwise.
     */
    virtual bool computeTrajectory() = 0;

    /**
     * Set the DCM reference.
     * @return true in case of success, false otherwise.
     */
    virtual bool setDCMReference(Eigen::Ref<const Eigen::MatrixXd> dcmReference);

    /**
     * Destructor.
     */
    virtual ~DCMPlanner() = default;
};
} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_DCM_PLANNER_H
