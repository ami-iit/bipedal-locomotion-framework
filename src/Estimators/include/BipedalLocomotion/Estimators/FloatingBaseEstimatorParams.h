/**
 * @file FloatingBaseEstimatorParams.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_PARAMS_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_PARAMS_H

#include <Eigen/Dense>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace FloatingBaseEstimators
{
/**
* @brief Struct containing sensor measurement deviation parameters of floating base estimators
*
*/
struct SensorsStdDev
{
    /**
    * @brief Additive white Gaussian noise deviation for accelerometer measurements
    *        Expressed in local frame as m/(s^2), in continuous time
    */
    Eigen::Vector3d accelerometerNoise;

    /**
    * @brief Additive white Gaussian noise deviation for gyroscope measurements
    *        Expressed in local frame as rad/s, in continuous time
    */
    Eigen::Vector3d gyroscopeNoise;

    /**
    * @brief Random walk bias noise deviation for accelerometer measurements
    *        Expressed in local frame as m/s^3, in continuous time
    */
    Eigen::Vector3d accelerometerBiasNoise;

    /**
    * @brief Random walk bias noise deviation for gyroscope measurements
    *        Expressed in local frame as rad/s^2, in continuous time
    */
    Eigen::Vector3d gyroscopeBiasNoise;

    /**
    * @brief White Gaussian noise deviation for linear feet velocities in rigid contact with the environment
    *        Expressed in local frame as m/s^2, in continuous time
    */
    Eigen::Vector3d contactFootLinvelNoise;

    /**
    * @brief White Gaussian noise deviation for angular feet velocities in rigid contact with the environment
    *        Expressed in local frame as rad/s, in continuous time
    */
    Eigen::Vector3d contactFootAngvelNoise;

    /**
    * @brief White Gaussian noise deviation for linear feet velocities in swing phase
    *        Expressed in local frame as m/s^2, in continuous time
    */
    Eigen::Vector3d swingFootLinvelNoise;

    /**
    * @brief White Gaussian noise deviation for angular feet velocities in swing phase
    *        Expressed in local frame as rad/s, in continuous time
    */
    Eigen::Vector3d swingFootAngvelNoise;

    /**
    * @brief White Gaussian noise deviation for relative kinematics between IMU frame to foot contact frames
    *        Expressed in local frame, in continuous time
    */
    Eigen::Matrix<double, 6, 1> forwardKinematicsNoise;

    /**
    * @brief White Gaussian noise deviation for encoder measurements in continuous time
    */
    Eigen::VectorXd encodersNoise;
};

/**
* @brief Struct containing prior state deviation parameters of floating base estimators
*
*/
struct StateStdDev
{
    /**
    * @brief Prior deviation of IMU orientation in inertial frame
    *        expressed in radians as rpy
    */
    Eigen::Vector3d imuOrientation;

    /**
    * @brief Prior deviation of IMU position in inertial frame
    *        expressed in m as xyz
    */
    Eigen::Vector3d imuPosition;

    /**
    * @brief Prior deviation of mixed-trivialized IMU linear velocity
    *        expressed in m/s
    */
    Eigen::Vector3d imuLinearVelocity;

    /**
    * @brief Prior deviation of left foot contact frame orientation in inertial frame
    *        expressed in radians as rpy
    */
    Eigen::Vector3d lContactFrameOrientation;

    /**
    * @brief Prior deviation of left foot contact frame position in inertial frame
    *        expressed in m as xyz
    */
    Eigen::Vector3d lContactFramePosition;

    /**
    * @brief Prior deviation of right foot contact frame orientation in inertial frame
    *        expressed in radians as rpy
    */
    Eigen::Vector3d rContactFrameOrientation;

    /**
    * @brief Prior deviation of right foot contact frame position in inertial frame
    *        expressed in m as xyz
    */
    Eigen::Vector3d rContactFramePosition;

    /**
    * @brief Prior deviation of IMU accelerometer bias in local frame
    *        expressed in m/s^2
    *
    */
    Eigen::Vector3d accelerometerBias;

    /**
    * @brief Prior deviation of IMU gyroscope bias in local frame
    *        expressed in rad/s
    */
    Eigen::Vector3d gyroscopeBias;
};


/**
* @brief Struct containing options runtime options for floating base estimator
*
*/
struct Options
{
    /**
    * @brief Enable/disable online accelerometer and gyroscope bias estimation
    *
    */
    bool imuBiasEstimationEnabled{false};

    /**
    * @brief Enable/disable IMU bias initialization using
    *        initial set of measurements in a static configuration
    * @note also remember to set nr_samples_for_bias_initialization
    *
    */
    bool staticImuBiasInitializationEnabled{false};

    /**
    * @brief Number of initial measurement samples in a static configuration
    *        used for IMU bias initialization
    *
    */
    int nrSamplesForImuBiasInitialization{0};

    /**
    * @brief Enable/disable measurement update step of the internal EKF
    *
    */
    bool ekfUpdateEnabled{true};

    /**
    * @brief Acceleration vector due to gravity
    *
    */
    Eigen::Vector3d accelerationDueToGravity{0, 0, -9.80665};
};

} //namespace FloatingBaseEstimators
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_PARAMS_H
