/**
 * @file PositionToTorqueController.h
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_JOINT_LEVEL_CONTROLLERS_POSITION_TO_TORQUE_CONTROLLER_H
#define BIPEDAL_LOCOMOTION_JOINT_LEVEL_CONTROLLERS_POSITION_TO_TORQUE_CONTROLLER_H

#include <memory>
#include <optional>

#include <Eigen/Dense>

#include <BipedalLocomotion/JointLevelControllers/PositionToJointController.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace JointLevelControllers
{

// clang-format off
/**
 * @brief Position-to-torque Controller
 *
 * The PositionToTorqueController implements a position control strategy that directly
 * computes joint torque commands from position errors.
 *
 * The controller implements a proportional-derivative position control law:
 * \f[
 *    \tau = K_p \cdot (q_{ref} - q_{fb}) + K_d \cdot (0 - \dot{q}_{fb})
 * \f]
 * where:
 * - \f$ \tau \f$ is the output torque [Nm]
 * - \f$ K_p \f$ is the proportional gain [Nm/rad]
 * - \f$ K_d \f$ is the derivative gain [Nm/(rad/s)]
 * - \f$ q_{ref} \f$ is the reference position [rad]
 * - \f$ q_{fb} \f$ is the feedback position [rad]
 * - \f$ \dot{q}_{fb} \f$ is the feedback velocity [rad/s]
 *
 */
class PositionToTorqueController: public PositionToJointController
{
    // clang-format on
public:
    /**
     * @brief Default constructor
     */
    PositionToTorqueController();

    /**
     * @brief Default destructor
     */
    virtual ~PositionToTorqueController();

    /**
     * @brief Advance the controller
     * @param input The input to the controller
     * @return The output of the controller
     */
    bool advance() override;

    // clang-format off
    /**
     * @brief Initialize the advanceable
     * @param handler A weak pointer to the parameters handler
     * @note The parameters handler should contain the following parameters:
     * |     Parameter Name      |      Type        |                             Description                           | Mandatory |
     * |:-----------------------:|:----------------:|:-----------------------------------------------------------------:|:---------:|
     * |    `joints_list`        | `vector<string>` |                             List of joints to control             |    Yes    |
     * Moreover the following group parameters are required:
     * |  Group Name    |                                                        Description                                           | Mandatory |
     * |:--------------:|:------------------------------------------------------------------------------------------------------------:|:---------:|
     * |      `kp`      |     An element for each joint contained in the `joints_list` parameter. Proportional gain for each joint     |    Yes    |
     * |      `kd`      |   An element for each joint contained in the `joints_list` parameter. Derivative gain for each joint         |    Yes    |
     *
     * Furthermore, the following optional parameters are supported:
     * |       Group Name      |                                                        Description                                        | Mandatory |
     * |:---------------------:|:---------------------------------------------------------------------------------------------------------:|:---------:|
     * |     `torque_limit`    | An element for each joint contained in the `joints_list` parameter. Torque limit for each joint [Nm]      |    No     |
     * @return True if the initialization is successful.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;
    // clang-format on

    /**
     * Set the input of the port
     * @param input the input of the controller
     * @return True in case of success and false otherwise
     */
    bool setInput(const PositionToJointControllerInput& input) override;

    /**
     * Get the output of the controller
     * @return The output of the controller
     */
    Eigen::VectorXd& getOutput() const override;

    /**
     * Check if the output is valid
     * @return True if the output is valid, false otherwise
     */
    bool isOutputValid() const override;

private:
    struct Impl; /**< Forward declaration of the implementation details */
    std::unique_ptr<Impl> m_pimpl; /**< Pointer to the implementation details */
};

} // namespace JointLevelControllers
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_POSITION_TO_TORQUE_CONTROLLER_H
