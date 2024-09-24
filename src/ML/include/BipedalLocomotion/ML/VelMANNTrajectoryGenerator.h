/**
 * @file VelMANNTrajectoryGenerator.h
 * @authors Evelyn D'Elia
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ML_VEL_MANN_TRAJECTORY_GENERATOR_H
#define BIPEDAL_LOCOMOTION_ML_VEL_MANN_TRAJECTORY_GENERATOR_H

#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/ML/VelMANNAutoregressive.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace ML
{

/**
 * Input of the planner.
 */
struct VelMANNTrajectoryGeneratorInput : public VelMANNAutoregressiveInput
{
    /** Index to the merge point considered to attach the new trajectory */
    std::size_t mergePointIndex;
};

/**
 * Output of the trajectory planner
 */
struct VelMANNTrajectoryGeneratorOutput
{
    /** CoM trajectory expressed in the inertial frame */
    std::vector<Eigen::Vector3d> comTrajectory;

    /** CoM velocity trajectory expressed in the inertial frame */
    std::vector<Eigen::Vector3d> comVelocityTrajectory;

    /** base velocity trajectory expressed in mixed representation */
    std::vector<manif::SE3d::Tangent> baseVelocityTrajectory;

    /** Centroidal angular momentum trajectory expressed in the mixed representation. */
    std::vector<Eigen::Vector3d> angularMomentumTrajectory;

    std::vector<Eigen::VectorXd> jointPositions; /**< Joints position in radians */
    std::vector<Eigen::VectorXd> jointVelocities; /**< Joint velocities in radians */
    std::vector<manif::SE3d> basePoses; /**< Vector containing the base pose for each instant. */

    std::vector<std::chrono::nanoseconds> timestamps; /**< Vector containing the time stamps. */

    Contacts::ContactPhaseList phaseList; /**< List of the contact phases */
};

/**
 * VelMANNTrajectoryGenerator is a class that uses VelMANNAutoregressive to generate a kinematically
 * feasible trajectory for humanoid robots. The planner will generate a trajectory which duration is
 * equal to `slow_down_factor * time_horizon` seconds. This class differs from the MANNTrajectoryGenerator
 * class in that the input and output features of the learned model are velocity-based rather than
 * position-based. The postprocessing, using the 3D velocity output features of the learned model, allows
 * more modularity in the trajectory generation. More details are available in the paper mentioned below.
 * @subsection mann_trajectory_generator VelMANN trajectory generator
 * The diagram illustrates the utilization of the VelMANNAutoregressive within the
 * VelMANNTrajectoryGenerator class.
 * To initialize the generator, the user needs to set the initial
 * state using the VelMANNTrajectoryGenerator::setInitialState method. The
 * VelMANNTrajectoryGeneratorInput, provided by the user, serves as the input for the
 * VelMANNAutoregressiveInput, along with the 'mergePointIndex.' The VelMANNAutoregressiveInput is assumed
 * to remain constant within the trajectory horizon. The 'mergePointIndex' indicates the index at
 * which the new trajectory will be attached. For example, when the VelMANNTrajectoryGenerator
 * generates a trajectory consisting of 'N' points, if the 'mergePointIndex' is set to 3, the first
 * three elements of the new trajectory will be derived from the previously computed trajectory by
 * VelMANNTrajectoryGeneratorOutput, utilizing the previous VelMANNAutoregressiveInput. Subsequently, all
 * points from 3 to N will be evaluated using the current VelMANNAutoregressiveInput. This behavior is
 * facilitated by a mechanism that stores the autoregressive state required for resetting the
 * VelMANNAutoregressive at the designated merge point. Every time the VelMANNAutoregressive::advance()
 * function is invoked by the VelMANNTrajectoryGenerator, the autoregressive state is stored for future
 * reference.
 * @note The implementation of the class follows the work presented in "Trajectory Generation with
 * Physics-Informed Learning and Drift Mitigation", available at
 * https://github.com/ami-iit/paper_delia_2024_ral_physics-informed_trajectory_generation.
 */
class VelMANNTrajectoryGenerator
    : public System::Advanceable<VelMANNTrajectoryGeneratorInput, VelMANNTrajectoryGeneratorOutput>
{
public:
    /**
     * Constructor
     */
    VelMANNTrajectoryGenerator();

    /**
     * Destructor
     */
    ~VelMANNTrajectoryGenerator();

    /**
     * Set the robot model.
     * @param model model of the robot considered by the network. Please load the very same model
     * with the same joint serialization used to train the VelMANN network.
     * @return true in case of success, false otherwise.
     */
    bool setRobotModel(const iDynTree::Model& model);

    // clang-format off
    /**
     * Initialize the trajectory planner.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |      Parameter Name      |   Type   |                                                  Description                                                  | Mandatory |
     * |:------------------------:|:--------:|:-------------------------------------------------------------------------------------------------------------:|:---------:|
     * |      `time_horizon`      | `double` |                                           Horizon of the planner.                                             |    Yes    |
     * |     `sampling_time`      | `double` |                                    Sampling considered in the inference.                                      |    Yes    |
     * |  `root_link_frame_name`  | `string` |                                 Name of of the root link frame in the model.                                  |    Yes    |
     * | `chest_link_frame_name`  | `string` |                                 Name of of the chest link frame in the model.                                 |    Yes    |
     * | `right_foot_frame_name`  | `string` |                                 Name of of the right foot frame in the model.                                 |    Yes    |
     * |  `left_foot_frame_name`  | `string` |                                  Name of of the left foot frame in the model.                                 |    Yes    |
     * |    `forward_direction`   | `string` |  String containing 'x', 'y' or 'z' representing the foot link forward axis. Currently, only 'x' is supported. |    Yes    |
     * |    `slow_down_factor`    | `double` |         The `horizon` will be `horizon * slow_down_factor`. Same for the `sampling_time`.                     |    Yes    |
     * |     `scaling_factor`     | `double` |     Scaling factor considered in the generation of the CoM, base and contact points locations                 |    Yes    |
     * It is also required to define two groups `LEFT_FOOT` and `RIGHT_FOOT` that contains the following parameter
     * |      Parameter Name      |        Type       |                                        Description                                             | Mandatory |
     * |:------------------------:|:-----------------:|:----------------------------------------------------------------------------------------------:|:---------:|
     * |    `number_of_corners`   |        `int`      |                          Number of corners associated to the foot                              |    Yes    |
     * |       `corner_<i>`       |  `vector<double>` | Position of the corner expressed in the foot frame. I must be from 0 to number_of_corners - 1  |    Yes    |
     * Finally it also required to define a group named `VelMANN` that contains the following parameter
     * |      Parameter Name      |   Type   |                          Description                                | Mandatory |
     * |:------------------------:|:--------:|:-------------------------------------------------------------------:|:---------:|
     * |    `onnx_model_path`     | `string` |  Path to the `onnx` model that will be loaded to perform inference  |    Yes    |
     * | `projected_base_horizon` |  `int`   |    Number of samples of the base horizon considered in the model    |    Yes    |
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

    /**
     * Set the initial state of the planner.
     * @param jointPositions position of the joints.
     * @param lefFoot status of the left foot.
     * @param lefFoot status of the right foot.
     * @param basePose pose of the base.
     * @param time initial time of the planner.
     */
    bool setInitialState(Eigen::Ref<const Eigen::VectorXd> jointPositions, //
                         const manif::SE3d& basePose);

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace ML
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ML_VEL_MANN_TRAJECTORY_GENERATOR_H
