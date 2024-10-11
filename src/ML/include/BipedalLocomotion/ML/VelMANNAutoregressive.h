/**
 * @file VelMANNAutoregressive.h
 * @authors Evelyn D'Elia
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ML_VEL_MANN_AUTOREGRESSIVE_H
#define BIPEDAL_LOCOMOTION_ML_VEL_MANN_AUTOREGRESSIVE_H

#include <chrono>
#include <deque>
#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/ML/VelMANN.h>
#include <BipedalLocomotion/Math/SchmittTrigger.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <iDynTree/KinDynComputations.h>

namespace BipedalLocomotion
{
namespace ML
{

/**
 * VelMANNFootState contains the state of a foot in contact with the ground.
 */
struct VelMANNFootState
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
     * @return a dummy VelMANNFootState.
     * @note a dummy VelMANNFootState is generated assuming that the foot is in contact with the ground
     * and the Math::SchmittTriggerState is in the on state (i.e., the foot is in contact). Moreover
     * a dummy VelMANNFootState is generated setting to zero the switch time of the Schmitt trigger.
     * @warning This function assumes that the foot is in contact with the ground and the
     * Math::SchmittTriggerState is in the on state (i.e., the foot is in contact).
     */
    static VelMANNFootState generateFootState(iDynTree::KinDynComputations& kindyn,
                                           const std::vector<Eigen::Vector3d>& corners,
                                           const std::string& footName,
                                           int footIndex);
};

/**
 * VelMANNAutoregressiveInput contains the unput to VelMANN network when used in autoregressive fashion.
 * The base position trajectory, base direction trajectory and base velocity trajectories are
 * written in a bidimensional local reference frame L in which we assume all the quantities related
 * to the ground-projected base trajectory in xi and yi to be expressed. At each step ti,
 * L is defined to have its origin in the current ground-projected robot base position and
 * orientation defined by the current base direction (along with its orthogonal vector).
 */
struct VelMANNAutoregressiveInput
{
    /** Matrix containing the future desired position trajectory. The rows contain the x and y
     * position projected into the ground while the columns the position at each time instant. */
    Eigen::Matrix2Xd desiredFutureBaseTrajectory;

    /** Matrix containing the desired future base direction trajectory. The rows contain the x and
     * y direction projected into the ground while the columns the direction at each time instant.
     */
    Eigen::Matrix2Xd desiredFutureBaseDirections;

    /** Matrix containing the desired base velocity trajectory. The rows contain the x and y
     * velocity projected into the ground while the columns the position at each time instant. */
    Eigen::Matrix2Xd desiredFutureBaseVelocities;

    /** Matrix containing the desired base angular velocity trajectory around the z axis (yaw). The columns denote the position at each time instant. */
    Eigen::RowVectorXd desiredFutureBaseAngVelocities;
};

/**
 * VelMANNAutoregressiveOutput contains the output to the VelMANN network when used in autoregressive
 * fashion.
 */
struct VelMANNAutoregressiveOutput
{
    Eigen::VectorXd jointsPosition; /**< Joint positions in radians */
    Eigen::VectorXd jointsVelocity; /**< Joint velocities in radians */
    manif::SE3d basePose; /**< Base pose with respect to the inertial frame, i.e., \f${}^I H_B\f$ */
    manif::SE3d::Tangent baseVelocity; /**< Base velocity in mixed representation */
    Eigen::Vector3d comPosition; /**< Position of the CoM with respect to the inertial frame */
    Eigen::Vector3d comVelocity; /**< Time derivative of the CoM position with respect to the inertial frame */
    Eigen::Vector3d angularMomentum; /**< Centroidal angular momentum */
    manif::SE3d leftFootPose; /**< Left foot pose with respect to the inertial frame, i.e., \f${}^I H_B\f$ */
    manif::SE3d rightFootPose; /**< Right foot pose with respect to the inertial frame, i.e., \f${}^I H_B\f$ */
    Eigen::VectorXd leftFootVelocity; /**< Left foot velocity with respect to the inertial frame, i.e., \f${}^I H_B\f$ */
    Eigen::VectorXd rightFootVelocity; /**< Right foot velocity with respect to the inertial frame, i.e., \f${}^I H_B\f$ */
    Contacts::EstimatedContact leftFoot; /**< Left foot contact */
    Contacts::EstimatedContact rightFoot; /**< Right foot contact */
    std::chrono::nanoseconds currentTime; /**< Current time stored in the advanceable */
};

// clang-format off
/**
 * VelMANNAutoregressive is a class that allows to perform autoregressive inference with Mode-Adaptive
 * Neural Networks (MANN).
 * @subsection VelMANN_autoregressive VelMANN Autoregressive
 * The following diagram shows how the VelMANN network is used inside the VelMANNAutoregressive class.
 * The output of VelMANN is given to two blocks, one computes the kinematically feasible base position,
 * while the other blends it to te input provided by the user
 * @note The implementation of the class follows the work presented in "Trajectory Generation with
 * Physics-Informed Learning and Drift Mitigation", available at
 * https://github.com/ami-iit/paper_delia_2024_ral_physics-informed_trajectory_generation.
 */
// clang-format on
class VelMANNAutoregressive
    : public System::Advanceable<VelMANNAutoregressiveInput, VelMANNAutoregressiveOutput>
{
public:
    /**
     * SupportFoot is an enum that contains the support foot considered by the VelMANN network.
     */
    enum class SupportFoot
    {
        Left, /**< Left foot */
        Right, /**< Right foot */
        Unknown /**< This is useful to detect unexpected behaviour */
    };

    /**
     * AutoregressiveState contains all quantities required to reset the Advanceable
     * The base position trajectory, base direction trajectory and base velocity trajectories are
     * written in a bidimensional local reference frame L in which we assume all the quantities
     * related to the ground-projected base trajectory in xi and yi to be expressed. At each step
     * ti, L is defined to have its origin in the current ground-projected robot base position and
     * orientation defined by the current base direction (along with its orthogonal vector).
     */
    struct AutoregressiveState
    {
        VelMANNOutput previousVelMannOutput; /**< Output of the VelMANN network generated at the previous
                                          iteration */
        VelMANNInput previousVelMannInput; /**< Input to the VelMANN network at previous iteration */
        manif::SE3d I_H_B_prev; /**< SE(3) transformation of the base direction respect to the
                               inertial frame*/

        /** Past base linear velocity, Each element of the deque contains the x, y, and z velocity. */
        std::deque<Eigen::Vector3d> pastProjectedBaseVelocity;

        /** Past base angular velocity, Each element of the deque contains x, y, and z angular velocity. */
        std::deque<Eigen::Vector3d> pastProjectedBaseAngVelocity;

        VelMANNFootState leftFootState; /**< Left foot state */
        VelMANNFootState rightFootState; /**< Right foot state */
        manif::SE3d I_H_B; /**< SE(3) transformation of the base with respect to the inertial frame */
        SupportFoot supportFoot{SupportFoot::Unknown}; /**< Support foot */
        Eigen::Vector3d projectedContactPositionInWorldFrame; /**< Projected contact position in
                                                                 world frame */
        int supportCornerIndex{-1}; /**< Index of the support corner */

        std::chrono::nanoseconds time; /**< Current time stored in the advanceable */

        /**< For the linear PID */
        Eigen::Vector2d I_x_des; /**< SE(3) transformation of the input reference base with respect to the inertial frame */

        /**< For the rotational PID */
        manif::SE3d I_H_ref; /**< SE(3) transformation of the input reference base with respect to the inertial frame */

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
         * projectedContactPositionInWorldFrame, zeroing the pastProjectedBaseVelocity and
         * pastProjectedBaseAngVelocity. The dummy AutoregressiveState set the
         * time to zero.
         */
        static AutoregressiveState
        generateDummyAutoregressiveState(const VelMANNInput& input,
                                         const VelMANNOutput& output,
                                         const manif::SE3d& I_H_B,
                                         const VelMANNFootState& leftFootState,
                                         const VelMANNFootState& rightFootState,
                                         int mocapFrameRate,
                                         const std::chrono::nanoseconds& pastProjectedBaseHorizon);
    };

    /**
     * Constructor
     */
    VelMANNAutoregressive();

    /**
     * Destructor
     */
    ~VelMANNAutoregressive();

    /**
     * Set the robot model.
     * @param model model of the robot considered by the network. Please load the very same model
     * with the same joint serialization used to train the VelMANN network.
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
     * |    Parameter Name    |       Type       |                                                  Description                                                  | Mandatory |
     * |:--------------------:|:----------------:|:-------------------------------------------------------------------------------------------------------------:|:---------:|
     * |  `number_of_corners` |       `int`      |                                   Number of corners associated to the foot                                    |    Yes    |
     * |      `corner_<i>`    | `vector<double>` |        Position of the corner expressed in the foot frame. It must be from 0 to number_of_corners - 1.        |    Yes    |
     * |     `on_threshold`   |     `double`     |      Distance between foot and ground used as threshold trigger to activate contact. It must be positive.     |    Yes    |
     * |    `off_threshold`   |     `double`     | Distance between the foot and the ground used as threshold trigger to deactivate contact. It must be positive.|    Yes    |
     * |   `switch_on_after`  |     `double`     | Seconds to wait before switching to activate from deactivate contact. Ensure it's greater than sampling time. |    Yes    |
     * |  `switch_off_after`  |     `double`     | Seconds to wait before switching to deactivate from activate contact. Ensure it's greater than sampling time. |    Yes    |
     * Finally it also required to define a group named `VelMANN` that contains the following parameter
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
     * @note Please call this function before calling VelMANNAutoregressive::advance the first time.
     * @warning This function assumes that both the feet are in contact with the ground, the joint
     * and base velocities equal to zero.
     * @warning This function reset also the internal time to zero.
     */
    bool reset(Eigen::Ref<const Eigen::VectorXd> jointPositions, const manif::SE3d& basePose);

    /**
     * Reset the autoregressive model
     * @param autoregressiveState the autoregressive state at which you want to reset your model.
     * @return true in case of success, false otherwise.
     * @note Please call this function the before calling VelMANNAutoregressive::advance the first time
     * time.
     */
    bool reset(const AutoregressiveState& autoregressiveState);

    /**
     * Check if the output is valid.
     * @return true if the output is valid, false otherwise.
     */
    bool isOutputValid() const override;

    /**
     * Get the output from VelMANNAutoregressive.
     * @return the output of the system.
     */
    const Output& getOutput() const override;

    /**
     * Get the structure that has been used as input for VelMANN.
     * @return the VelMANNInput
     */
    const VelMANNInput& getVelMANNInput() const;

    /**
     * Get the autoregressive state required to rest VelMANNAutoregressive.
     * @return the AutoregressiveState
     */
    const AutoregressiveState& getAutoregressiveState() const;

    /**
     * Get the autoregressive state required to reset VelMANNAutoregressive.
     * @param jointPositions joint position
     * @param basePose base pose
     * @param state autoregressive state that will be populated
     * @return true in case of success, false otherwise.
     * @note This function assumes that both the feet are in contact with the ground, the joint
     * and base velocities equal to zero.
     */
    bool populateInitialAutoregressiveState(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                                            const manif::SE3d& basePose,
                                            VelMANNAutoregressive::AutoregressiveState& state);

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace ML
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ML_VEL_MANN_AUTOREGRESSIVE_H
