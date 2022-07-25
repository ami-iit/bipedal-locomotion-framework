/**
 * @file FloatingBaseEstimatorParams.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_FLOATING_BASE_ESTIMATOR_PARAMS_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_FLOATING_BASE_ESTIMATOR_PARAMS_H

#include <Eigen/Dense>
#include <map>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace FloatingBaseEstimators
{

// see https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
using PoseCovariance = std::map<int, Eigen::Matrix<double, 6, 1>, std::less<int>,
          Eigen::aligned_allocator<std::pair<const int, Eigen::Matrix<double, 6, 1> > > >;

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

    /**
    * @brief White Gaussian noise deviation for landmark relative pose predictions in continuous time
    */
    Eigen::Matrix<double, 6, 1> landmarkPredictionNoise;

    /**
    * @brief White Gaussian noise deviation for landmark relative pose measurments in continuous time
    */
    Eigen::Matrix<double, 6, 1> landmarkMeasurementNoise;

    /**
    * @brief White Gaussian noise deviation for linear velocity of IMU frame wrt inertial frame
    *        Expressed in local frame as m/s, in continuous time
    */
    Eigen::Vector3d imuFrameLinearVelocityNoise;

    /**
    * @brief White Gaussian noise deviation for angular velocity of IMU frame wrt inertial frame
    *        Expressed in local frame as rad/s, in continuous time
    */
    Eigen::Vector3d imuFrameAngularVelocityNoise;
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
    * @brief Prior deviation of mixed-trivialized IMU angular velocity
    *        expressed in rad/s
    */
    Eigen::Vector3d imuAngularVelocity;

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

    /**
    * @brief Container of deviations of support frame pose in inertial frame
    */
    PoseCovariance supportFramePose;

    /**
    * @brief Container of deviations of landmark pose in inertial frame
    */
    PoseCovariance landmarkPose;
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
     * @brief Enable/disable kinematics based correction updates
     */
    bool kinematicsUpdateEnabled{true};

    /**
     * @brief Enable/disable landmarks based correction updates
     */
    bool staticLandmarksUpdateEnabled{false};


    /**
    * @brief Acceleration vector due to gravity
    *
    */
    Eigen::Vector3d accelerationDueToGravity{0, 0, -9.80665};
};

} //namespace FloatingBaseEstimators
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_FLOATING_BASE_ESTIMATOR_PARAMS_H
