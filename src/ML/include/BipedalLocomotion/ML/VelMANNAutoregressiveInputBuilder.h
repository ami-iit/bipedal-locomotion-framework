/**
 * @file VelMANNAutoregressiveInputBuilder.h
 * @authors Evelyn D'Elia
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ML_VEL_MANN_AUTOREGRESSIVE_INPUT_BUILDER_H
#define BIPEDAL_LOCOMOTION_ML_VEL_MANN_AUTOREGRESSIVE_INPUT_BUILDER_H

#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/ML/VelMANNAutoregressive.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace ML
{

/**
 * Structure representing directional input for VelMANNAutoregressive.
 * This structure holds the motion direction and base direction as 2D vectors.
 */

struct VelMANNDirectionalInput
{
    /**< The direction of motion. */
    Eigen::Vector2d motionDirection;

    /**< The direction the base is facing. */
    Eigen::Vector2d baseDirection;
};

/**
 * VelMANNAutoregressiveInputBuilder generates a VelMANNAutoregressiveInput from a pair of directional
 * inputs.
 */
class VelMANNAutoregressiveInputBuilder
    : public System::Advanceable<VelMANNDirectionalInput, VelMANNAutoregressiveInput>
{
public:
    /**
     * Constructor
     */
    VelMANNAutoregressiveInputBuilder();

    /**
     * Destructor
     */
    ~VelMANNAutoregressiveInputBuilder();

    // clang-format off
    /**
     * Initialize the trajectory builder..
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |                  Parameter Name                 |   Type   |                                           Description                                           | Mandatory |
     * |:-----------------------------------------------:|:--------:|:---------------------------------------------------------------------------------------------------------------------------------------------------------------:|:---------:|
     * |                 `base_vel_norm`                 | `double` |                             Norm of the desired base velocity in m/s                            |    Yes    |
     * |               `base_ang_vel_norm`               | `double` |                       Norm of the desired base angular velocity in rad/s                        |    Yes    |
     * |             `ellipsoid_forward_axis`            | `double` |                          Size of the forward axis ellipsoid in meters                           |    Yes    |
     * |            `ellipsoid_backward_axis`            | `double` |                         Size of the backward axis ellipsoid in meters                           |    Yes    |
     * |              `ellipsoid_side_axis`              | `double` |                          Size of the lateral axis ellipsoid in meters                           |    Yes    |
     * |            `ellipsoid_scaling_factor`           | `double` | Scaling factor considered in the generation of the final and control points of the Bezier Curve |    Yes    |
     * |        `max_base_direction_angle_forward`       | `double` |              Maximum angle of the base direction when the robot is walking forward              |    Yes    |
     * |       `max_base_direction_angle_backward`       | `double` |             Maximum angle of the base direction when the robot is walking backward              |    Yes    |
     * |  `max_base_direction_angle_side_opposite_sign`  | `double` |       Maximum base direction angle when robot walks sideways facing the walking direction       |    Yes    |
     * |    `max_base_direction_angle_side_same_sign`    | `double` |   Maximum base direction angle when robot walks sideways facing opposite the walking direction  |    Yes    |
     * |                `number_of_knots`                |   `int`  |                          Number of knots considered in the Bezier curve                         |    Yes    |
     * |          `forward_direction_threshold`          | `double` |  The minimum value for the user-specified motion direction to be interpreted as forward motion  |    No     |
     * |            `side_direction_threshold`           | `double` |    The minimum value for the user-specified motion direction to be interpreted as side motion   |    No     |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;
    // clang-format on

    /**
     * Set the input to the planner model.
     * @param input input to the model
     * @return true in case of success, false otherwise.
     */
    bool setInput(const Input& input) override;

    /**
     * Generate the trajectory
     * @return true in case of success, false otherwise.
     */
    bool advance() override;

    /**
     * Check if the output is valid.
     * @return true if the output is valid, false otherwise.
     */
    bool isOutputValid() const override;

    /**
     * Get the output from trajectory.
     * @return the output of the system.
     */
    const Output& getOutput() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace ML
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ML_VEL_MANN_AUTOREGRESSIVE_INPUT_BUILDER_H
