/**
 * @file TaskSpaceInverseDynamics.h
 * @authors Ines Sorrentino, Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TASK_SPACE_INVERSE_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_TASK_SPACE_INVERSE_DYNAMICS_H

#include <unordered_map>

#include <Eigen/Dense>
#include <manif/SE3.h>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/System/ILinearTaskSolver.h>
#include <BipedalLocomotion/TSID/TSIDLinearTask.h>



namespace BipedalLocomotion
{

namespace TSID
{

/**
 * State of the TaskSpaceInverseDynamics
 */
struct TSIDState
{
    manif::SE3d::Tangent baseAcceleration; /**< Mixed acceleration of the base */
    Eigen::VectorXd jointAccelerations; /**< Joints acceleration in rad per second per second */
    Eigen::VectorXd jointTorques; /**< Joint torques */

    /**< List of the information related to the contact wrenches */
    std::unordered_map<std::string, Contacts::ContactWrench> contactWrenches;
};

/**
 * TaskSpaceInverseDynamics implements the interface for the task space inverse
 * dynamics. Please inherit this class if you want to implement your custom Task TSID.
 * The TSIDState is a struct containing the joint acceleration, joint torques
 * and contact wrenches. The TaskSpaceInverseDynamics can be used to generate the desired joint
 * torques to be sent to the low-level torque controllers.
 */
class TaskSpaceInverseDynamics : public System::ILinearTaskSolver<TSIDLinearTask, TSIDState>
{
public:

    /**
     * Destructor.
     */
    virtual ~TaskSpaceInverseDynamics() = default;
};

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_TASK_SPACE_INVERSE_DYNAMICS_H
