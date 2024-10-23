/**
 * @file PINNFrictionEstimator.h
 * @authors Ines Sorrentino
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_PINN_FRICTION_ESTIMATOR_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_PINN_FRICTION_ESTIMATOR_H

#include <memory>
#include <mutex>
#include <yarp/sig/Vector.h>

/**
 * PINNFrictionEstimator is a class that performs joint friction torque estimation
 * using a physics informed neural network model.
 * This class uses NN models exported as ONNX files
 * Such NNs take 2 quantities as input:
 * - Joint position and its history
 * - Motor position and its history
 */
class PINNFrictionEstimator
{
public:
    PINNFrictionEstimator();
    ~PINNFrictionEstimator();

    /**
     * Initialize the estimator
     * @param[in] modelPath a string representing the path to the ONNX model
	 * @param[in] intraOpNumThreads a std::size_t representing the number of threads to be used for intra-op parallelism
	 * @param[in] interOpNumThreads a std::size_t representing the number of threads to be used for inter-op parallelism
     * @return true if the initialization is successful, false otherwise
     */
    bool initialize(const std::string& modelPath,
                    const std::size_t intraOpNumThreads = 1,
                    const std::size_t interOpNumThreads = 1);

    /**
     * Reset the estimator
     * This function clears the buffers
     * and resets the internal state of the estimator
     * @return void
     */
    void resetEstimator();

    /**
     * Estimate the joint friction starting from raw data
     * @param[in] inputMotorVelocity a double representing the motor velocity (rad/sec)
     * @param[in] inputJointVelocity a double representing the joint velocity (rad/sec)
     * @param[out] output a double representing the joint friction torque
     * @return true if the estimation is successful, false otherwise
     */
    bool estimate(double inputMotorVelocity,
                  double inputJointVelocity,
                  double& output);


private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_PINN_FRICTION_ESTIMATOR_H
