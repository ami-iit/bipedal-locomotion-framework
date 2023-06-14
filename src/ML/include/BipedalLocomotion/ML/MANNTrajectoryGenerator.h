/**
 * @file MANNTrajectoryGenerator.h
 * @authors Paolo Maria Viceconte, Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ML_MANN_TRAJECTORY_GENERATOR_H
#define BIPEDAL_LOCOMOTION_ML_MANN_TRAJECTORY_GENERATOR_H

#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/ML/MANNAutoregressive.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace ML
{

/**
 * Input of the planner.
 */
struct MANNTrajectoryGeneratorInput : public MANNAutoregressiveInput
{
    /** Index to the merge point considered to attach the new trajectory */
    std::size_t mergePointIndex;
};

/**
 * Output of the trajectory planner
 */
struct MANNTrajectoryGeneratorOutput
{
    /** CoM trajectory expressed in the inertial frame */
    std::vector<Eigen::Vector3d> comTrajectory;

    /** Centroidal angular momentum trajectory expressed in the mixed representation. */
    std::vector<Eigen::Vector3d> angularMomentumTrajectory;

    std::vector<Eigen::VectorXd> jointPositions; /**< Joints position in radians */
    std::vector<manif::SE3d> basePoses; /**< Vector containing the base pose for each instant. */
    Contacts::ContactPhaseList phaseList; /**< List of the contact phases */
};

/**
 * MANNTrajectoryGenerator is a class that uses MANNAutoregressive to generate a kinematically
 * feasible trajectory for humanoid robots. The planner will generate a trajectory which duration is
 * equal to `slow_down_factor * time_horizon` seconds.
 * @subsection mann_trajectory_generator MANN trajectory generator
 * The diagram illustrates the utilization of the MANNAutoregressive within the
 * MANNTrajectoryGenerator class.
 * <img src="https://user-images.githubusercontent.com/16744101/239922103-1f23e08b-0091-475a-8879-61af66399c62.png" alt="mann_trajectory_generator">
 * To initialize the generator, the user needs to set the initial
 * state using the MANNTrajectoryGenerator::setInitialState method. The
 * MANNTrajectoryGeneratorInput, provided by the user, serves as the input for the
 * MANNAutoregressiveInput, along with the 'mergePointIndex.' The MANNAutoregressiveInput is assumed
 * to remain constant within the trajectory horizon. The 'mergePointIndex' indicates the index at
 * which the new trajectory will be attached. For example, when the MANNTrajectoryGenerator
 * generates a trajectory consisting of 'N' points, if the 'mergePointIndex' is set to 3, the first
 * three elements of the new trajectory will be derived from the previously computed trajectory by
 * MANNTrajectoryGeneratorOutput, utilizing the previous MANNAutoregressiveInput. Subsequently, all
 * points from 3 to N will be evaluated using the current MANNAutoregressiveInput. This behavior is
 * facilitated by a mechanism that stores the autoregressive state required for resetting the
 * MANNAutoregressive at the designated merge point. Every time the MANNAutoregressive::advance()
 * function is invoked by the MANNTrajectoryGenerator, the autoregressive state is stored for future
 * reference.
 * @note The implementation of the class follows the work presented in "P. M. Viceconte et al.,
 * ADHERENT: Learning Human-like Trajectory Generators for Whole-body Control of Humanoid Robots,
 * in IEEE Robotics and Automation Letters, vol. 7, no. 2, pp. 2779-2786, April 2022,
 * doi: 10.1109/LRA.2022.3141658." https://doi.org/10.1109/LRA.2022.3141658
 */
class MANNTrajectoryGenerator
    : public System::Advanceable<MANNTrajectoryGeneratorInput, MANNTrajectoryGeneratorOutput>
{
public:
    /**
     * Constructor
     */
    MANNTrajectoryGenerator();

    /**
     * Destructor
     */
    ~MANNTrajectoryGenerator();

    /**
     * Set the robot model.
     * @param model model of the robot considered by the network. Please load the very same model
     * with the same joint serialization used to train the MANN network.
     * @return true in case of success, false otherwise.
     */
    bool setRobotModel(const iDynTree::Model& model);

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
     * |    `forward_direction`   | `string` | String cointaining 'x', 'y' or 'z' representing the foot link forward axis. Currently, only 'x' is supported. |    Yes    |
     * |    `slow_down_factor`    |   `int`  |      The `horizon` will be `horizon * slow_down_factor`. Same for the `sampling_time` (default value 1).      |    No     |
     * It is also required to define two groups `LEFT_FOOT` and `RIGHT_FOOT` that contains the following parameter
     * |      Parameter Name      |        Type       |                                        Description                                             | Mandatory |
     * |:------------------------:|:-----------------:|:----------------------------------------------------------------------------------------------:|:---------:|
     * |    `number_of_corners`   |        `int`      |                          Number of corners associated to the foot                              |    Yes    |
     * |       `corner_<i>`       |  `vector<double>` | Position of the corner expressed in the foot frame. I must be from 0 to number_of_corners - 1  |    Yes    |
     * Finally it also required to define a group named `MANN` that contains the following parameter
     * |      Parameter Name      |   Type   |                          Description                                | Mandatory |
     * |:------------------------:|:--------:|:-------------------------------------------------------------------:|:---------:|
     * |    `onnx_model_path`     | `string` |  Path to the `onnx` model that will be loaded to perform inference  |    Yes    |
     * | `projected_base_horizon` |  `int`   |    Number of samples of the base horizon considered in the model    |    Yes    |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;

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
    void setInitialState(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                         const Contacts::EstimatedContact& leftFoot,
                         const Contacts::EstimatedContact& rightFoot,
                         const manif::SE3d& basePose,
                         const std::chrono::nanoseconds& time);

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace ML
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ML_MANN_TRAJECTORY_GENERATOR_H
