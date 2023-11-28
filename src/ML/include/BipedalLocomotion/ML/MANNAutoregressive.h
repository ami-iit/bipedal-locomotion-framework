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
 * MANNFootState contains the state of a foot in contact with the ground.
 */
struct MANNFootState
{
    Contacts::EstimatedContact contact; /**< Contact state */
    Math::SchmittTriggerState schmittTriggerState; /**< Schmitt trigger state */
    std::vector<Eigen::Vector3d> corners; /**< Corners of the foot */

    /**
     * Generate a foot state.
     * @param kindyn an instance of KinDynComputations used to compute the foot position.
     * @param corners Vector containing the corners of the foot expressed in the foot frame (i.e., a
     * frame attached to the foot sole with the x axis pointing forward, the y axis pointing to the
     * left and the z axis pointing upward).
     * @param footName name of the foot.
     * @param footIndex index of the foot in the model. This is required to compute the foot
     * position with the KinDynComputations object.
     * @return a dummy MANNFootState.
     * @note a dummy MANNFootState is generated assuming that the foot is in contact with the ground
     * and the Math::SchmittTriggerState is in the on state (i.e., the foot is in contact). Moreover
     * a dummy MANNFootState is generated setting to zero the switch time of the Schmitt trigger.
     * @warning This function assumes that the foot is in contact with the ground and the
     * Math::SchmittTriggerState is in the on state (i.e., the foot is in contact).
     */
    static MANNFootState generateFootState(iDynTree::KinDynComputations& kindyn,
                                           const std::vector<Eigen::Vector3d>& corners,
                                           const std::string& footName,
                                           int footIndex);
};

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
    Eigen::Vector3d comPosition; /**< Position of the CoM with respect to the inertial frame */
    Eigen::Vector3d angularMomentum; /**< Centroidal angular momentum */
    Contacts::EstimatedContact leftFoot; /**< Left foot contact */
    Contacts::EstimatedContact rightFoot; /**< Right foot contact */
    std::chrono::nanoseconds currentTime; /**< Current time stored in the advanceable */
};

// clang-format off
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
// clang-format on
class MANNAutoregressive
    : public System::Advanceable<MANNAutoregressiveInput, MANNAutoregressiveOutput>
{
public:
    /**
     * SupportFoot is an enum that contains the support foot considered by the MANN network.
     */
    enum class SupportFoot
    {
        Left, /**< Left foot */
        Right, /**< Right foot */
        Unknown /**< This is useful to detect unexpected behaviour */
    };

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
        MANNOutput previousMANNOutput; /**< Output of the MANN network generated at the previous
                                          iteration */
        MANNInput previousMANNInput; /**< Input to the MANN network at previous iteration */
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

        MANNFootState leftFootState; /**< Left foot state */
        MANNFootState rightFootState; /**< Right foot state */
        manif::SE3d I_H_B; /**< SE(3) transformation of the base respect to the inertial frame */
        SupportFoot supportFoot{SupportFoot::Unknown}; /**< Support foot */
        Eigen::Vector3d projectedContactPositionInWorldFrame; /**< Projected contact position in
                                                                 world frame */
        int supportCornerIndex{-1}; /**< Index of the support corner */

        std::chrono::nanoseconds time; /**< Current time stored in the advanceable */

        /**
         * Generate a dummy autoregressive state from the input.
         * @param input input to the autoregressive model.
         * @param output output of the autoregressive model.
         * @param I_H_B SE(3) transformation of the base respect to the inertial frame.
         * @param leftFootState left foot state.
         * @param rightFootState right foot state.
         * @param mocapFrameRate frame rate of the mocap data.
         * @param pastProjectedBaseHorizon number of samples of the past base horizon considered in
         * the neural network.
         * @return A dummy AutoregressiveState.
         * @note A dummy AutoregressiveState is generated zeroing the
         * projectedContactPositionInWorldFrame, zeroing the pastProjectedBasePositions and
         * pastProjectedBaseVelocity. On the other hand, the pastFacingDirection is generated with
         * the first row equal to one and the second to zero. The dummy AutoregressiveState set the
         * time to zero.
         */
        static AutoregressiveState
        generateDummyAutoregressiveState(const MANNInput& input,
                                         const MANNOutput& output,
                                         const manif::SE3d& I_H_B,
                                         const MANNFootState& leftFootState,
                                         const MANNFootState& rightFootState,
                                         int mocapFrameRate,
                                         const std::chrono::nanoseconds& pastProjectedBaseHorizon);
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

    // clang-format off
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
     * |    `forward_direction`   | `string` |  String containing 'x', 'y' or 'z' representing the foot link forward axis. Currently, only 'x' is supported. |    Yes    |
     * |    `mocap_frame_rate`    |   `int`  |                                       Frame rate of the mocap data.                                           |    Yes    |
     * | `past_projected_horizon` | `double` |                    Number of seconds of the past base horizon considered in the neural network.               |    Yes    |
     * It is also required to define two groups `LEFT_FOOT` and `RIGHT_FOOT` that contains the following parameter
     * |    Parameter Name    |       Type       |                                                        Description                                                           | Mandatory |
     * |:--------------------:|:----------------:|:----------------------------------------------------------------------------------------------------------------------------:|:---------:|
     * |  `number_of_corners` |       `int`      |                                        Number of corners associated to the foot                                              |    Yes    |
     * |      `corner_<i>`    | `vector<double>` |               Position of the corner expressed in the foot frame. It must be from 0 to number_of_corners - 1.                |    Yes    |
     * |     `on_threshold`   |     `double`     |  Distance between the foot and the ground used as on threshold of the trigger to activate the contact. It must be positive.  |    Yes    |
     * |    `off_threshold`   |     `double`     | Distance between the foot and the ground used as off threshold of the trigger to deactivate the contact. It must be positive.|    Yes    |
     * |   `switch_on_after`  |     `double`     |     Seconds to wait for before switching to activate from deactivate contact. Ensure it's greater than sampling time.        |    Yes    |
     * |  `switch_off_after`  |     `double`     |     Seconds to wait for before switching to deactivate from activate contact. Ensure it's greater than sampling time.        |    Yes    |
     * Finally it also required to define a group named `MANN` that contains the following parameter
     * |          Parameter Name         |   Type   |                                         Description                                         | Mandatory |
     * |:-------------------------------:|:--------:|:-------------------------------------------------------------------------------------------:|:---------:|
     * |        `onnx_model_path`        | `string` |         Path to the `onnx` model that will be loaded to perform inference.                  |    Yes    |
     * |   `projected_base_datapoints`   |  `int`   | Number of samples of the base horizon considered in the model (It must be an even number).  |    Yes    |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;
    // clang-format on

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
     * @param jointPositions joint position.
     * @param basePose base pose.
     * @return true in case of success, false otherwise.
     * @note Please call this function before calling MANNAutoregressive::advance the first time.
     * @warning This function assumes that both the feet are in contact with the ground, the joint
     * and base velocities equal to zero.
     * @warning This function reset also the internal time to zero.
     */
    bool reset(Eigen::Ref<const Eigen::VectorXd> jointPositions, const manif::SE3d& basePose);

    /**
     * Reset the autoregressive model
     * @param autoregressiveState the autoregressive state at which you want to reset your model.
     * @return true in case of success, false otherwise.
     * @note Please call this function the before calling MANNAutoregressive::advance the first time
     * time.
     */
    bool reset(const AutoregressiveState& autoregressiveState);

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

    /**
     * Get the autoregressive state required to reset MANNAutoregressive.
     * @param jointPositions joint position
     * @param basePose base pose
     * @param state autoregressive state that will be populated
     * @return true in case of success, false otherwise.
     * @note This function assumes that both the feet are in contact with the ground, the joint
     * and base velocities equal to zero.
     */
    bool populateInitialAutoregressiveState(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                                            const manif::SE3d& basePose,
                                            MANNAutoregressive::AutoregressiveState& state);

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace ML
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ML_MANN_AUTOREGRESSIVE_H
