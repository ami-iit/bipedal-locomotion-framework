/**
 * @file PositionToCurrentController.h
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_JOINT_LEVEL_CONTROLLERS_POSITION_TO_CURRENT_CONTROLLER_H
#define BIPEDAL_LOCOMOTION_JOINT_LEVEL_CONTROLLERS_POSITION_TO_CURRENT_CONTROLLER_H

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
 * @brief Input data structure for the PositionToCurrentController
 *
 * This structure contains all the feedback and reference signals required by the controller
 * to compute the appropriate motor current commands. All vectors must have the same size
 * corresponding to the number of controlled joints.
 */
struct PositionToCurrentControllerInput
{
    Eigen::VectorXd referencePosition; /**< Desired joint positions [rad] */
    Eigen::VectorXd feedbackPosition; /**< Current joint positions from encoders [rad] */
    Eigen::VectorXd feedbackVelocity; /**< Current joint velocities from encoders [rad/s] */
};

// clang-format off
/**
 * @brief Position-to-Current Controller with Friction Compensation and TN-Curve Limiting
 *
 * The PositionToCurrentController implements a position control strategy that directly
 * computes motor current commands from position errors. This approach is particularly useful for
 * robots where direct current control provides better performance than traditional position control
 * cascades, especially in applications requiring precise force/torque control or friction compensation.
 *
 * @note Controller Architecture
 *
 * The controller implements a proportional position control law with Coulomb friction feedforward
 * compensation and dynamic current limiting based on motor torque-speed (TN) characteristics:
 *
 * \f[
 *     I = \text{clamp}\left(\frac{K_p \cdot (q_{ref} - q_{fb}) + \tau_{coulomb}}{\text{gearRatio} \cdot k_{\tau}}, I_{limit}(\omega)\right)
 * \f]
 *
 * where:
 * - \f$ I \f$ is the output current [A]
 * - \f$ K_p \f$ is the proportional gain [Nm/rad]
 * - \f$ q_{ref} \f$ is the reference position [rad]
 * - \f$ q_{fb} \f$ is the feedback position [rad]
 * - \f$ \tau_{coulomb} = k_{coulomb} \cdot \text{sign}(\dot{q}_{fb}) \f$ is the Coulomb friction compensation [Nm]
 * - \f$ \dot{q}_{fb} \f$ is the feedback velocity [rad/s]
 * - \f$ \text{gearRatio} \f$ is the gear reduction ratio (motor-to-joint)
 * - \f$ k_{\tau} \f$ is the motor torque constant [Nm/A]
 * - \f$ I_{limit}(\omega) \f$ is the velocity-dependent current limit [A]
 *
 * ## Friction Compensation
 *
 * The controller includes feedforward Coulomb friction compensation that adds a torque
 * in the direction of motion to counteract coulomb friction effects:
 *
 * \f[
 *     \tau_{coulomb} = k_{coulomb} \cdot \text{tanh}(\dot{q}_{fb} / v_{activation})
 * \f]
 *
 * where:
 * - \f$ k_{coulomb} \f$ is the Coulomb friction constant [Nm]
 * - \f$ v_{activation} \f$ is the velocity threshold for friction activation [rad/s]
 *
 * This helps improve tracking accuracy, especially at low velocities where friction effects
 * are dominant.
 *
 * ## Dynamic Current Limiting (TN-Curve)
 *
 * The controller implements velocity-dependent current limiting based on motor torque-speed
 * characteristics to prevent motor overheating and ensure safe operation:
 *
 * - **Constant Region** (\f$ |\omega| \leq \omega_{rated} \f$): \f$ I_{limit} = I_{max} \f$
 * - **Linear Falloff** (\f$ \omega_{rated} < |\omega| < \omega_{0} \f$): \f$ I_{limit} = m \cdot |\omega| + b \f$
 * - **No-Load Speed** (\f$ |\omega| \geq \omega_{0} \f$): \f$ I_{limit} = 0 \f$
 *
 * where:
 * - \f$ \omega_{rated} \f$ is the rated speed at which current limiting begins [rad/s]
 * - \f$ \omega_{0} \f$ is the no-load speed where current becomes zero [rad/s]
 * - \f$ m = -I_{max} / (\omega_{0} - \omega_{rated}) \f$ is the slope of the linear region
 * - \f$ b = I_{max} - m \cdot \omega_{rated} \f$ is the intercept
 *
 */
class PositionToCurrentController
    : public BipedalLocomotion::System::Advanceable<PositionToCurrentControllerInput,
                                                    Eigen::VectorXd>
{
    // clang-format on
public:
    /**
     * @brief Default constructor
     */
    PositionToCurrentController();

    /**
     * @brief Default destructor
     */
    virtual ~PositionToCurrentController();

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
     * | `current_safety_factor` |     `double`     | Value used to scale the current limits (0, 1] (Default value 1.0) |    No     |
     * Moreover the following group parameters are required:
     * |  Group Name    |                                                        Description                                           | Mandatory |
     * |:--------------:|:------------------------------------------------------------------------------------------------------------:|:---------:|
     * |      `kp`      |     An element for each joint contained in the `joints_list` parameter. Proportional gain for each joint     |    Yes    |
     * |   `gearbox`    | An element for each joint contained in the `joints_list` parameter. Gear ratio for each joint (motor->joint) |    Yes    |
     * |    `k_tau`     |   An element for each joint contained in the `joints_list` parameter. Torque constant for each joint [Nm/A]  |    Yes    |
     *
     * Furthermore, the following optional parameters are supported:
     * |       Group Name      |                                                        Description                                        | Mandatory |
     * |:---------------------:|:---------------------------------------------------------------------------------------------------------:|:---------:|
     * |     `current_limit`   | An element for each joint contained in the `joints_list` parameter. Current limit for each joint [A]      |    No     |
     * |   `coulomb_friction`  | An element for each joint contained in the `joints_list` parameter. Coulomb friction for each joint [Nm]  |    No     |
     * | `activation_velocity` | An element for each joint contained in the `joints_list` parameter. Friction activation velocity [rad/s]  |    No     |
     * |    `rated_speed`      | An element for each joint contained in the `joints_list` parameter. Rated speed for each joint [rad/s]    |    No     |
     * |    `no_load_speed`    | An element for each joint contained in the `joints_list` parameter. No-load speed for each joint [rad/s]  |    No     |
     * @return True if the initialization is successful.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;
    // clang-format on

    /**
     * Set the input of the port
     * @param input the input of the controller
     * @return True in case of success and false otherwise
     */
    bool setInput(const PositionToCurrentControllerInput& input) override;

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

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_POSITION_TO_CURRENT_CONTROLLER_H
