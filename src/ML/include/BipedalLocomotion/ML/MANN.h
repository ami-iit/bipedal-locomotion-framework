/**
 * @file MANN.h
 * @authors Paolo Maria Viceconte, Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ML_MANN_H
#define BIPEDAL_LOCOMOTION_ML_MANN_H

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
 * MANNInput contains the input to the MANN network.
 * The base position trajectory, facing direction trajectory and base velocity trajectories are
 * written in a bidimensional local reference frame L in which we assume all the quantities related
 * to the ground-projected base trajectory in xi and yi to be expressed. At each step ti,
 * L is defined to have its origin in the current ground-projected robot base position and
 * orientation defined by the current facing direction (along with its orthogonal vector).
 */
struct MANNInput
{
    /** Matrix containing the base position trajectory. The rows contain the x and y position
     * projected into the ground while the columns the position at each time instant. */
    Eigen::Matrix2Xd basePositionTrajectory;
    /** Matrix containing the facing direction trajectory. The rows contain the x and y direction
     * projected into the ground while the columns the direction at each time instant. */
    Eigen::Matrix2Xd facingDirectionTrajectory;
    /** Matrix containing the base velocity trajectory. The rows contain the x and y velocity
     *  projected into the ground while the columns the position  at each time instant. */
    Eigen::Matrix2Xd baseVelocitiesTrajectory;

    Eigen::VectorXd jointPositions; /**< Vector containing the actual joint position in radians. */
    Eigen::VectorXd jointVelocities; /**< Vector containing the actual joint velocity in radians per
                                        seconds. */

    /**
     * Generate a dummy MANNInput from a given joint configuration
     * @param jointPositions vector containing the joint position in radians.
     * @param projectedBaseHorizon number of samples of the base horizon considered in the neural
     * network.
     * @return a dummy MANNInput.
     * @note A dummy MANNInput is generated assuming zero joint velocities and zero
     * basePositionTrajectory, zero baseVelocitiesTrajectory. and facingDirectionTrajectory with the
     * first row equal to one and the second to zero.
     * @warning the function assumes that the robot is not moving and facing forward.
     */
    static MANNInput generateDummyMANNInput(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                                            std::size_t projectedBaseHorizon);
};

/**
 * MANNOutput contains the output to the MANN network.
 * The base position trajectory, facing direction trajectory and base velocity trajectories are
 * written in a bidimensional local reference frame L in which we assume all the quantities related
 * to the ground-projected base trajectory in xi and yi to be expressed. At each step ti,
 * L is defined to have its origin in the current ground-projected robot base position and
 * orientation defined by the current facing direction (along with its orthogonal vector).
 */
struct MANNOutput
{
    /** Matrix containing the future base position trajectory. The rows contains the x and y
     * position projected into the ground while the columns the position at each time
     * instant. */
    Eigen::Matrix2Xd futureBasePositionTrajectory;

    /** Matrix containing the future base facing direction. The rows contains the x and y direction
     * projected into the ground while the columns the position at each time instant. */
    Eigen::Matrix2Xd futureFacingDirectionTrajectory;

    /** Matrix containing the future base velocity trajectory. The rows contains the x and y
     * velocity projected into the ground while the columns the position at each time instant. */
    Eigen::Matrix2Xd futureBaseVelocitiesTrajectory;
    Eigen::VectorXd jointPositions; /**< Vector containing the next joint position in radians */
    Eigen::VectorXd jointVelocities; /**< Vector containing the next joint velocity in radians per
                                        seconds */
    manif::SE2Tangentd projectedBaseVelocity; /**< Velocity of the base projected on the ground */

    /**
     * Generate a dummy MANNOutput from a given joint configuration
     * @param jointPositions vector containing the joint positions in radians.
     * @param futureProjectedBaseHorizon number of samples of the base horizon generated as output
     * by the neural network.
     * @return a dummy MANNOutput.
     * @note A dummy MANNOutput is generated assuming zero joint velocities, zero
     * futureBasePositionTrajectory, zero futureBaseVelocitiesTrajectory, and
     * futureFacingDirectionTrajectory with the first row equal to one and the second to zero.
     * @warning the function assumes that the robot is not moving and facing forward.
     */
    static MANNOutput generateDummyMANNOutput(Eigen::Ref<const Eigen::VectorXd> jointPositions,
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
class MANN : public BipedalLocomotion::System::Advanceable<MANNInput, MANNOutput>
{
public:
    /**
     * Constructor
     */
    MANN();

    /**
     * Destructor. It is required by the pimpl implementation.
     */
    ~MANN();

    // clang-format off
    /**
     * Initialize the network.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |        Parameter Name       |       Type       |                                       Description                                               | Mandatory |
     * |:---------------------------:|:----------------:|:-----------------------------------------------------------------------------------------------:|:---------:|
     * |       `onnx_model_path`     |      `string`    |          Path to the `onnx` model that will be loaded to perform inference                      |    Yes    |
     * |     `number_of_joints`      |       `int`      |               Number of robot joints considered in the model                                    |    Yes    |
     * | `projected_base_datapoints` |       `int`      |    Number of samples of the base horizon considered in the model (it must be an even number)    |    Yes    |
     * |     `number_of_threads`     |       `int`      | Number of threads used to perform the inference. Default value is the number of available cores |    No     |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;
    // clang-format on

    /**
     * Set the input of the network
     * @param input the struct containing all the inputs of the network.
     * @return true if case of success, false otherwise.
     */
    bool setInput(const MANNInput& input) override;

    /**
     * Perform one step of the inference given the input set.
     * @return true if case of success, false otherwise.
     */
    bool advance() override;

    /**
     * Get the output of the network once advance has been called.
     * @return the output of the network.
     */
    const MANNOutput& getOutput() const override;

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

#endif // BIPEDAL_LOCOMOTION_ML_MANN_H
