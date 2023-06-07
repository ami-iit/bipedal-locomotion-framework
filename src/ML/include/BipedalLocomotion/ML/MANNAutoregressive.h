/**
 * @file MANNAutoregressive.h
 * @authors Paolo Maria Viceconte, Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ML_MANN_AUTOREGRESSIVE_H
#define BIPEDAL_LOCOMOTION_ML_MANN_AUTOREGRESSIVE_H

#include <chrono>
#include <deque>
#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/ML/MANN.h>
#include <BipedalLocomotion/Math/SchmittTrigger.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <iDynTree/KinDynComputations.h>

namespace BipedalLocomotion
{
namespace ML
{

/**
 * MANNAutoregressiveInput contains the unput to MANN network when used in autoregressive fashion.
 * The base position trajectory, facing direction trajectory and base velocity trajectories are
 * written in a bidimensional local reference frame L in which we assume all the quantities related
 * to the ground-projected base trajectory in xi and yi to be expressed. At each step ti,
 * L is defined to have its origin in the current ground-projected robot base position and
 * orientation defined by the current facing direction (along with its orthogonal vector).
 */
struct MANNAutoregressiveInput
{
    /** Matrix containing the future desired position trajectory. The rows contain the x and y
     * position projected into the ground while the columns the position at each time instant. */
    Eigen::Matrix2Xd desiredFutureBaseTrajectory;

    /** Matrix containing the desired future facing direction trajectory. The rows contain the x and
     * y direction projected into the ground while the columns the direction at each time instant.
     */
    Eigen::Matrix2Xd desiredFutureFacingDirections;

    /** Matrix containing the desired base velocity trajectory. The rows contain the x and y
     * velocity projected into the ground while the columns the position  at each time instant. */
    Eigen::Matrix2Xd desiredFutureBaseVelocities;
};

/**
 * MANNAutoregressiveOutput contains the output to the MANN network when used in autoregressive
 * fashion.
 */
struct MANNAutoregressiveOutput
{
    Eigen::VectorXd jointsPosition; /**< Joint positions in radians */
    manif::SE3d basePose; /**< Base pose with respect to the inertial frame, i.e., \f${}^I H_B\f$ */
    manif::SE3d::Tangent baseVelocity; /**< Base velocity in mixed representation */
    Eigen::Vector3d comPosition;
    Eigen::Vector3d angularMomentum;
    Contacts::EstimatedContact leftFoot; /**< Left foot contact */
    Math::SchmittTriggerState leftFootSchmittTriggerState;
    Contacts::EstimatedContact rightFoot; /**< Right foot contact */
    Math::SchmittTriggerState rightFootSchmittTriggerState;
    std::chrono::nanoseconds currentTime; /**< Current time stored in the advanceable */
};

/**
 * MANNAutoregressive is a class that allows to perform autoregressive inference with Mode-Adaptive
 * Neural Networks (MANN).
 * @subsection mann_autoregressive MANN Autoregressive
 * The following diagram shows how the MANN network is used inside the MANNAutoregressive class.
 * The output of MANN is given to two blocks, one computes the kinematically feasible base position,
 * while the other blends it to te input provided by the user
 * <img src="https://user-images.githubusercontent.com/16744101/237064989-b3d0d62f-2fc9-4ca4-9b11-ecca0c8c378e.png" alt="mann_autoregressive">
 * @note The implementation of the class follows the work presented in "P. M. Viceconte et al.,
 * "ADHERENT: Learning Human-like Trajectory Generators for Whole-body Control of Humanoid Robots,"
 * in IEEE Robotics and Automation Letters, vol. 7, no. 2, pp. 2779-2786, April 2022,
 * doi: 10.1109/LRA.2022.3141658." https://doi.org/10.1109/LRA.2022.3141658
 */
class MANNAutoregressive
    : public System::Advanceable<MANNAutoregressiveInput, MANNAutoregressiveOutput>
{
public:
    /**
     * AutoregressiveState contains all quantities required to reset the Advanceable
     * The base position trajectory, facing direction trajectory and base velocity trajectories are
     * written in a bidimensional local reference frame L in which we assume all the quantities
     * related to the ground-projected base trajectory in xi and yi to be expressed. At each step
     * ti, L is defined to have its origin in the current ground-projected robot base position and
     * orientation defined by the current facing direction (along with its orthogonal vector).
     */
    struct AutoregressiveState
    {
        MANNInput previousMannInput; /**< Mann Input at the previous time instant */
        manif::SE2d I_H_FD; /**< SE(2) transformation of the facing direction respect to the
                               inertial frame*/

        /** Past projected base position, Each element of the deque contains the x and y position
         * projected into the ground. */
        std::deque<Eigen::Vector2d> pastProjectedBasePositions;

        /** Past projected facing direction, Each element of the deque contains the x and y
         * direction  projected into the ground. */
        std::deque<Eigen::Vector2d> pastFacingDirection;

        /** Past projected base velocity, Each element of the deque contains the x and y velocity
         * projected into the ground. */
        std::deque<Eigen::Vector2d> pastProjectedBaseVelocity;
    };

    /**
     * Constructor
     */
    MANNAutoregressive();

    /**
     * Destructor
     */
    ~MANNAutoregressive();

    /**
     * Set the robot model.
     * @param model model of the robot considered by the network. Please load the very same model
     * with the same joint serialization used to train the MANN network.
     * @return true in case of success, false otherwise.
     */
    bool setRobotModel(const iDynTree::Model& model);

    /**
     * Initialize the network.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |      Parameter Name      |   Type   |                                                  Description                                                  | Mandatory |
     * |:------------------------:|:--------:|:-------------------------------------------------------------------------------------------------------------:|:---------:|
     * |     `sampling_time`      | `double` |                                    Sampling considered in the inference.                                      |    Yes    |
     * |  `root_link_frame_name`  | `string` |                                 Name of of the root link frame in the model.                                  |    Yes    |
     * | `chest_link_frame_name`  | `string` |                                 Name of of the chest link frame in the model.                                 |    Yes    |
     * | `right_foot_frame_name`  | `string` |                                 Name of of the right foot frame in the model.                                 |    Yes    |
     * |  `left_foot_frame_name`  | `string` |                                  Name of of the left foot frame in the model.                                 |    Yes    |
     * |    `forward_direction`   | `string` | String cointaining 'x', 'y' or 'z' representing the foot link forward axis. Currently, only 'x' is supported. |    Yes    |
     * It is also required to define two groups `LEFT_FOOT` and `RIGHT_FOOT` that contains the following parameter
     * |    Parameter Name    |       Type       |                                                        Description                                                           | Mandatory |
     * |:--------------------:|:----------------:|:----------------------------------------------------------------------------------------------------------------------------:|:---------:|
     * |  `number_of_corners` |       `int`      |                                        Number of corners associated to the foot                                              |    Yes    |
     * |      `corner_<i>`    | `vector<double>` |               Position of the corner expressed in the foot frame. It must be from 0 to number_of_corners - 1.                 |    Yes    |
     * |     `on_threshold`   |     `double`     |  Distance between the foot and the ground used as on threshold of the trigger to activate the contact. It must be positive.  |    Yes    |
     * |    `off_threshold`   |     `double`     | Distance between the foot and the ground used as off threshold of the trigger to deactivate the contact. It must be positive. |    Yes    |
     * |   `switch_on_after`  |     `double`     |     Seconds to wait for before switching to activate from deactivate contact. Ensure it's greater than sampling time.        |    Yes    |
     * |  `switch_off_after`  |     `double`     |     Seconds to wait for before switching to deactivate from activate contact. Ensure it's greater than sampling time.        |    Yes    |
     * Finally it also required to define a group named `MANN` that contains the following parameter
     * |      Parameter Name      |   Type   |                          Description                                | Mandatory |
     * |:------------------------:|:--------:|:-------------------------------------------------------------------:|:---------:|
     * |    `onnx_model_path`     | `string` |  Path to the `onnx` model that will be loaded to perform inference. |    Yes    |
     * | `projected_base_horizon` |  `int`   |    Number of samples of the base horizon considered in the model.   |    Yes    |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;

    /**
     * Set the input to the autoregressive model.
     * @param input input to the model
     * @return true in case of success, false otherwise.
     */
    bool setInput(const Input& input) override;

    /**
     * Perform one step of the autoregressive model
     * @return true in case of success, false otherwise.
     */
    bool advance() override;

    /**
     * Reset the autoregressive model
     * @param input raw mann input.
     * @param leftFoot state of the left foot.
     * @param rightFoot state of the right foot.
     * @param basePosition current base position.
     * @param baseVelocity current base velocity expressed in mixed representation.
     * @return true in case of success, false otherwise.
     * @note please call this function the before calling MANNAutoregressive::advance the first time
     * time.
     */
    bool reset(const MANNInput& input,
               const Contacts::EstimatedContact& leftFoot,
               const Contacts::EstimatedContact& rightFoot,
               const manif::SE3d& basePose,
               const manif::SE3Tangentd& baseVelocity);

    /**
     * Reset the autoregressive model
     * @param input raw mann input.
     * @param leftFoot state of the left foot.
     * @param rightFoot state of the right foot.
     * @param basePosition current base position.
     * @param baseVelocity current base velocity expressed in mixed representation.
     * @param autoregressiveState the autoregressive state at which you want to reset your model.
     * @param time time used to reset the system
     * @return true in case of success, false otherwise.
     * @note please call this function the before calling MANNAutoregressive::advance the first time
     * time.
     */
    bool reset(const MANNInput& input,
               const Contacts::EstimatedContact& leftFoot,
               const Math::SchmittTriggerState& leftFootSchimittTriggerState,
               const Contacts::EstimatedContact& rightFoot,
               const Math::SchmittTriggerState& rightFootSchimittTriggerState,
               const manif::SE3d& basePosition,
               const manif::SE3Tangentd& baseVelocity,
               const AutoregressiveState& autoregressiveState,
               const std::chrono::nanoseconds& time);

    /**
     * Check if the output is valid.
     * @return true if the output is valid, false otherwise.
     */
    bool isOutputValid() const override;

    /**
     * Get the output from MANNAutoregressive.
     * @return the output of the system.
     */
    const Output& getOutput() const override;

    /**
     * Get the structure that has been used as input for MANN.
     * @return the MANNInput
     */
    const MANNInput& getMANNInput() const;

    /**
     * Get the autoregressive state required to rest MANNAutoregressive.
     * @return the AutoregressiveState
     */
    const AutoregressiveState& getAutoregressiveState() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace ML
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ML_MANN_AUTOREGRESSIVE_H
