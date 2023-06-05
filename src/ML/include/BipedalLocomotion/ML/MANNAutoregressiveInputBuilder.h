/**
 * @file MANNAutoregressiveInputBuilder.h
 * @authors Paolo Maria Viceconte, Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ML_MANN_AUTOREGRESSIVE_INPUT_BUILDER_H
#define BIPEDAL_LOCOMOTION_ML_MANN_AUTOREGRESSIVE_INPUT_BUILDER_H

#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/ML/MANNAutoregressive.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace ML
{

/**
 * Structure representing directional input for MANNAutoregressive.
 * This structure holds the motion direction and facing direction as 2D vectors.
 */
struct MANNDirectionalInput
{
    /**< The direction of motion. */
    Eigen::Vector2d motionDirection;

    /**< The direction the agent is facing. */
    Eigen::Vector2d facingDirection;
};

/**
 * MANNAutoregressiveInputBuilder generates a MANNAutoregressiveInput from a pair of directional
 * inputs.
 */
class MANNAutoregressiveInputBuilder
    : public System::Advanceable<MANNDirectionalInput, MANNAutoregressiveInput>
{
public:
    /**
     * Constructor
     */
    MANNAutoregressiveInputBuilder();

    /**
     * Destructor
     */
    ~MANNAutoregressiveInputBuilder();

    // clang-format off
    /**
     * Initialize the trajectory builder..
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |                  Parameter Name                 |   Type   |                                                             Description                                                             | Mandatory |
     * |:-----------------------------------------------:|:--------:|:-----------------------------------------------------------------------------------------------------------------------------------:|:---------:|
     * |                 `base_vel_norm`                 | `double` |                                              Norm of the desired base velocity in m/s                                               |    Yes    |
     * |             `ellipsoid_forward_axis`            | `double` |                                             Size of the forward axis ellipsoid in meters                                            |    Yes    |
     * |            `ellipsoid_backward_axis`            | `double` |                                            Size of the backward axis ellipsoid in meters                                            |    Yes    |
     * |              `ellipsoid_side_axis`              | `double` |                                             Size of the lateral axis ellipsoid in meters                                            |    Yes    |
     * |            `ellipsoid_scaling_factor`           | `double` |                   Scaling factor considered in the generation of the final and control points of the Bezier Curve                   |    Yes    |
     * |       `max_facing_direction_angle_forward`      | `double` |                               Maximum angle of the facing direction when the robot is walking forward                               |    Yes    |
     * |      `max_facing_direction_angle_backward`      | `double` |                               Maximum angle of the facing direction when the robot is walking backward                              |    Yes    |
     * | `max_facing_direction_angle_side_opposite_sign` | `double` |         Maximum angle of the facing direction when the robot is walking lateral and is facing towards the walking direction         |    Yes    |
     * |   `max_facing_direction_angle_side_same_sign`   | `double` | Maximum angle of the facing direction when the robot is walking lateral and is facing towards the opposite of the walking direction |    Yes    |
     * |                `number_of_knots`                |   `int`  |                                            Number of knots considered in the Bezier curve                                           |    Yes    |
     * |          `forward_direction_threshold`          | `double` |     The minimum value for the normalized user-specified motion direction to be interpreted as forward motion (Default value 0.2)    |    No     |
     * |            `side_direction_threshold`           | `double` |       The minimum value for the normalized user-specified motion direction to be interpreted as side motion (Default value 0.2)     |    No     |
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

#endif // BIPEDAL_LOCOMOTION_ML_MANN_AUTOREGRESSIVE_INPUT_BUILDER_H
