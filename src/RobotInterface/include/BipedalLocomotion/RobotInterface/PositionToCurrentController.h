/**
 * @file PositionToCurrentController.h
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_POSITION_TO_CURRENT_CONTROLLER_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_POSITION_TO_CURRENT_CONTROLLER_H

#include <memory>
#include <optional>

#include <Eigen/Dense>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

namespace BipedalLocomotion::RobotInterface
{

/**
 * Input structure for the PositionToCurrentController.
 * It contains the reference position, feedback position, and feedback velocity
 * for the controller.
 */
struct PositionToCurrentControllerInput
{
    Eigen::VectorXd referencePosition; /**< Reference position for the controller */
    Eigen::VectorXd feedbackPosition; /**< Feedback position for the controller */
    Eigen::VectorXd feedbackVelocity; /**< Feedback velocity for the controller */
};

// clang-format off
/**
 * Output structure for the PositionToCurrentController.
 * The controller computes the output current based on the input
 * reference position, feedback position, and feedback velocity.
 * It implements a proportional controller with feedforward terms
 * for torque and friction compensation.
 * The output is the current in Amperes for each joint.
 * The output is computed as:
 * \f[
 *     I = \frac{K_p \cdot (q_{ref} - q_{fb}) + k_{coulomb} \cdot \text{sign}(v_{fb})}{\text{gearRatio} \cdot k_{\tau}}
 * \f]
 * where:
 * - \(I\) is the output current
 * - \(K_p\) is the proportional gain
 * - \(q_{ref}\) is the reference position
 * - \(q_{fb}\) is the feedback position
 * - \(v_{fb}\) is the feedback velocity
 * - \(k_{coulomb}\) is the Coulomb friction
 * - \(\text{gearRatio}\) is the gear ratio for the controller
 * - \(k_{\tau}\) is the torque constant for the controller
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
     * |   Parameter Name     |        Type                |         Description         | Mandatory |
     * |:--------------------:|:--------------------------:|:---------------------------:|:---------:|
     * | joints_list          | `std::vector<std::string>` | List of joints to control   |    Yes    |
     * Moreover the following group parameters are required:
     * |  Group Name    |                                                        Description                                           | Mandatory |
     * |:--------------:|:------------------------------------------------------------------------------------------------------------:|:---------:|
     * |      kp        |     An element for each joint contained in the `joints_list` parameter. Proportional gain for each joint     |    Yes    |
     * |   gearbox      | An element for each joint contained in the `joints_list` parameter. Gear ratio for each joint (motor->joint) |    Yes    |
     * |    k_tau       |   An element for each joint contained in the `joints_list` parameter. Torque constant for each joint [Nm/A]  |    Yes    |
     * Furthermore, the following optional parameters are supported:
     * |    Group Name    |                                                        Description                                        | Mandatory |
     * |:----------------:|:---------------------------------------------------------------------------------------------------------:|:---------:|
     * |   current_limit  | An element for each joint contained in the `joints_list` parameter. Current limit for each joint [A]      |    No     |
     * | coulomb_friction | An element for each joint contained in the `joints_list` parameter. Coulomb friction for each joint [Nm]  |    No     |
     * |   rated_speed    | An element for each joint contained in the `joints_list` parameter. Rated speed for each joint [rad/s]    |    No     |
     * |   no_load_speed  | An element for each joint contained in the `joints_list` parameter. No-load speed for each joint [rad/s]  |    No     |
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

} // namespace BipedalLocomotion::RobotInterface

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_POSITION_TO_CURRENT_CONTROLLER_H
