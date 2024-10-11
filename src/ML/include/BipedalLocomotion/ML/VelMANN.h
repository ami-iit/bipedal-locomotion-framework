/**
 * @file VelMANN.h
 * @authors Evelyn D'Elia
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ML_VEL_MANN_H
#define BIPEDAL_LOCOMOTION_ML_VEL_MANN_H

#include <Eigen/Dense>
#include <manif/SE2.h>

#include <memory>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

namespace BipedalLocomotion
{
namespace ML
{

/**
 * VelMANNInput contains the input to the MANN network.
 * The base linear and angular velocity trajectories are
 * written in a tridimensional local reference frame. At each step ti,
 * L is defined to have its origin in the current robot base position and
 * orientation.
 */
struct VelMANNInput
{
    /** Matrix containing the base linear velocity trajectory. The rows contain the xyz linear velocity
     * while the columns represent the velocity at each time instant. */
    Eigen::Matrix3Xd baseLinearVelocityTrajectory;
    /** Matrix containing the base angular velocity trajectory. The rows contain the xyz angular velocity
     * while the columns represent the velocity at each time instant. */
    Eigen::Matrix3Xd baseAngularVelocityTrajectory;

    Eigen::VectorXd jointPositions; /**< Vector containing the actual joint position in radians. */
    Eigen::VectorXd jointVelocities; /**< Vector containing the actual joint velocity in radians per
                                        seconds. */
    Eigen::Vector3d basePosition; /**< Vector containing the actual base position in m */
    Eigen::Vector3d baseAngle; /**< Vector containing the actual base euler angles in radians */

    /**
     * Generate a dummy VelMANNInput from a given joint configuration
     * @param jointPositions vector containing the joint position in radians.
     * @param projectedBaseHorizon number of samples of the base horizon considered in the neural
     * network.
     * @return a dummy VelMANNInput.
     * @note A dummy VelMANNInput is generated assuming zero joint velocities, zero
     * baseLinearVelocityTrajectory, and zero baseAngularVelocityTrajectory.
     * @warning the function assumes that the robot is not moving and facing forward.
     */
    static VelMANNInput generateDummyVelMANNInput(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                                            std::size_t projectedBaseHorizon);
};

/**
 * MANNOutput contains the output to the MANN network.
 * The base linear and angular velocity trajectories are
 * written in a tridimensional local reference frame. At each step ti,
 * L is defined to have its origin in the current robot base position and
 * orientation.
 */
struct VelMANNOutput
{
    /** Matrix containing the future base linear velocity trajectory. The rows contain the xyz linear velocity
     * while the columns represent the velocity at each time instant. */
    Eigen::Matrix3Xd futureBaseLinearVelocityTrajectory;

    /** Matrix containing the future base angular velocity trajectory. The rows contain the xyz angular velocity
     * while the columns represent the velocity at each time instant. */
    Eigen::Matrix3Xd futureBaseAngularVelocityTrajectory;

    Eigen::VectorXd jointPositions; /**< Vector containing the next joint position in radians */
    Eigen::VectorXd jointVelocities; /**< Vector containing the next joint velocity in radians per
                                        seconds */

    Eigen::Vector3d basePosition; /**< Vector containing the next base position in m */
    Eigen::Vector3d baseAngle; /**< Vector containing the next base euler angles in radians */

    /**
     * Generate a dummy MANNOutput from a given joint configuration
     * @param jointPositions vector containing the joint positions in radians.
     * @param futureProjectedBaseHorizon number of samples of the base horizon generated as output
     * by the neural network.
     * @return a dummy MANNOutput.
     * @note A dummy MANNOutput is generated assuming zero joint velocities, zero
     * futureBaseLinearVelocityTrajectory, and zero futureBaseAngularVelocityTrajectory.
     * @warning the function assumes that the robot is not moving and facing forward.
     */
    static VelMANNOutput generateDummyVelMANNOutput(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                                              std::size_t futureProjectedBaseHorizon);
};

/**
 * MANN is a class that allows to perform the inference of an `onnx` implementing Mode-Adaptive
 * Neural Networks (MANN). The network was originally proposed in [H. Zhang, S. Starke, T. Komura,
 * and J. Saito, “Mode-adaptive neural networks for quadruped motion control,” ACM Trans. Graph.,
 * vol. 37, no. 4, pp. 1–11, 2018.](https://doi.org/10.1145/3197517.3201366) This class uses
 * `onnxruntime` to perform inference and it has been tested with an `onnx` model generated from
 * https://github.com/ami-iit/mann-pytorch
 */
class VelMANN : public BipedalLocomotion::System::Advanceable<VelMANNInput, VelMANNOutput>
{
public:
    /**
     * Constructor
     */
    VelMANN();

    /**
     * Destructor. It is required by the pimpl implementation.
     */
    ~VelMANN();

    // clang-format off
    /**
     * Initialize the network.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |        Parameter Name       |       Type       |                                       Description                                              | Mandatory |
     * |:---------------------------:|:----------------:|:----------------------------------------------------------------------------------------------:|:---------:|
     * |       `onnx_model_path`     |      `string`    |          Path to the `onnx` model that will be loaded to perform inference                     |    Yes    |
     * |     `number_of_joints`      |       `int`      |               Number of robot joints considered in the model                                   |    Yes    |
     * | `projected_base_datapoints` |       `int`      |    Number of samples of the base horizon considered in the model (it must be an even number)   |    Yes    |
     * |     `number_of_threads`     |       `int`      |Number of threads used to perform the inference. Default value is the number of available cores |    No     |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;
    // clang-format on

    /**
     * Set the input of the network
     * @param input the struct containing all the inputs of the network.
     * @return true in case of success, false otherwise.
     */
    bool setInput(const VelMANNInput& input) override;

    /**
     * Perform one step of the inference given the input set.
     * @return true in case of success, false otherwise.
     */
    bool advance() override;

    /**
     * Get the output of the network once advance has been called.
     * @return the output of the network.
     */
    const VelMANNOutput& getOutput() const override;

    /**
     * Check if the output of the network is valid.
     * @return true if it is valid, false otherwise.
     */
    bool isOutputValid() const override;

private:
    /** Private implementation */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace ML
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ML_VEL_MANN_H
