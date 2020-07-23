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

/**
* @brief Struct containing sensor measurement deviation parameters of floating base estimators
*
*/
class FBESensorsStdDev
{
    /**
    * @brief Additive white Gaussian noise deviation for accelerometer measurements
    *        Expressed in local frame as m/(s^2), in continuous time
    */
    Eigen::Vector3d accelerometer_noise;

    /**
    * @brief Additive white Gaussian noise deviation for gyroscope measurements
    *        Expressed in local frame as rad/s, in continuous time
    */
    Eigen::Vector3d gyroscope_noise;

    /**
    * @brief Random walk bias noise deviation for accelerometer measurements
    *        Expressed in local frame as m/s^3, in continuous time
    */
    Eigen::Vector3d accelerometer_bias_noise;

    /**
    * @brief Random walk bias noise deviation for gyroscope measurements
    *        Expressed in local frame as rad/s^2, in continuous time
    */
    Eigen::Vector3d gyroscope_bias_noise;

    /**
    * @brief White Gaussian noise deviation for linear feet velocities in rigid contact with the environment
    *        Expressed in local frame as m/s^2, in continuous time
    */
    Eigen::Vector3d contact_foot_linvel_noise;

    /**
    * @brief White Gaussian noise deviation for angular feet velocities in rigid contact with the environment
    *        Expressed in local frame as rad/s, in continuous time
    */
    Eigen::Vector3d contact_foot_angvel_noise;

    /**
    * @brief White Gaussian noise deviation for linear feet velocities in swing phase
    *        Expressed in local frame as m/s^2, in continuous time
    */
    Eigen::Vector3d swing_foot_linvel_noise;

    /**
    * @brief White Gaussian noise deviation for angular feet velocities in swing phase
    *        Expressed in local frame as rad/s, in continuous time
    */
    Eigen::Vector3d swing_foot_angvel_noise;

    /**
    * @brief White Gaussian noise deviation for relative kinematics between IMU frame to foot contact frames
    *        Expressed in local frame, in continuous time
    *
    */
    Eigen::Matrix<double, 6, 1> forward_kinematics_noise;
};

/**
* @brief Struct containing prior state deviation parameters of floating base estimators
*
*/
struct FBEPriorsStdDev
{
    /**
    * @brief Prior deviation of IMU orientation in inertial frame
    *        expressed in radians as rpy
    */
    Eigen::Vector3d imu_orientation;

    /**
    * @brief Prior deviation of IMU position in inertial frame
    *        expressed in m as xyz
    */
    Eigen::Vector3d imu_position;

    /**
    * @brief Prior deviation of mixed-trivialized IMU linear velocity
    *        expressed in m/s
    */
    Eigen::Vector3d imu_linear_velocity;

    /**
    * @brief Prior deviation of left foot contact frame orientation in inertial frame
    *        expressed in radians as rpy
    */
    Eigen::Vector3d l_contact_frame_orientation;

    /**
    * @brief Prior deviation of left foot contact frame position in inertial frame
    *        expressed in m as xyz
    */
    Eigen::Vector3d l_contact_frame_position;

    /**
    * @brief Prior deviation of right foot contact frame orientation in inertial frame
    *        expressed in radians as rpy
    */
    Eigen::Vector3d r_contact_frame_orientation;

    /**
    * @brief Prior deviation of right foot contact frame position in inertial frame
    *        expressed in m as xyz
    */
    Eigen::Vector3d r_contact_frame_position;

    /**
    * @brief Prior deviation of IMU accelerometer bias in local frame
    *        expressed in m/s^2
    *
    */
    Eigen::Vector3d accelerometer_bias;

    /**
    * @brief Prior deviation of IMU gyroscope bias in local frame
    *        expressed in rad/s
    */
    Eigen::Vector3d gyroscope_bias;
};


/**
* @brief Struct containing options runtime options for floating base estimator
*
*/
struct FBEOptions
{
    /**
    * @brief Enable/disable online accelerometer and gyroscope bias estimation
    *
    */
    bool enable_imu_bias_estimation{false};

    /**
    * @brief Enable/disable IMU bias initialization using
    *        initial set of measurements in a static configuration
    * @note also remember to set nr_samples_for_bias_initialization
    *
    */
    bool enable_static_imu_bias_initialization{false};

    /**
    * @brief Number of initial measurement samples in a static configuration
    *        used for IMU bias initialization
    *
    */
    int nr_samples_for_imu_bias_initialization{0};

    /**
    * @brief Enable/disable measurement update step of the internal EKF
    *
    */
    bool enable_ekf_update{true};
};


} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_PARAMS_H
