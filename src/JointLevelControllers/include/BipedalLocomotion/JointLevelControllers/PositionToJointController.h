/**
 * @file PositionToJointController.h
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_JOINT_LEVEL_CONTROLLERS_POSITION_TO_JOINT_CONTROLLER_H
#define BIPEDAL_LOCOMOTION_JOINT_LEVEL_CONTROLLERS_POSITION_TO_JOINT_CONTROLLER_H

#include <memory>
#include <optional>

#include <Eigen/Dense>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace JointLevelControllers
{

/**
 * @brief Input data structure for the PositionToJointController
 *
 * This structure contains all the feedback and reference signals required by the controller
 * to compute the appropriate motor joint commands. All vectors must have the same size
 * corresponding to the number of controlled joints.
 */
struct PositionToJointControllerInput
{
    Eigen::VectorXd referencePosition; /**< Desired joint positions [rad] */
    Eigen::VectorXd feedbackPosition; /**< Current joint positions from encoders [rad] */
    Eigen::VectorXd feedbackVelocity; /**< Current joint velocities from encoders [rad/s] */
};

/**
 * Base class for Position-to-Joint Controllers.
 */
class PositionToJointController
    : public BipedalLocomotion::System::Advanceable<PositionToJointControllerInput, Eigen::VectorXd>
{
};

} // namespace JointLevelControllers
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_POSITION_TO_JOINT_CONTROLLER_H
