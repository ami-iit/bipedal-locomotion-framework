/**
 * @file FloatingBaseEstimatorIO.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_IO_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_IO_H

#include <Eigen/Dense>

namespace BipedalLocomotion
{
namespace Estimators
{

/**
* @brief Struct holding the elements of the state representation
*
*/
struct FBEInternalState
{


    Eigen::Quaterniond A_q_imu; /**< Orientation of the base link IMU in the inertial frame (as a quaternion w,x,y,z) */
    Eigen::Vector3d A_p_imu;   /**< Position of the base link IMU in the inertial frame */
    Eigen::Vector3d IMUA_v_AIMU; /**< linear part of the mixed-velocity representation of the IMU with respect to the inertial frame */
    Eigen::Quaterniond A_q_LF; /**< Orientation of the left foot contact frame in the inertial frame (as a quaternion w,x,y,z) */
    Eigen::Vector3d A_p_LF; /**< Position of the left foot contact frame in the inertial frame */
    Eigen::Quaterniond A_q_RF; /**< Orientation of the right foot contact frame in the inertial frame (as a quaternion w,x,y,z) */
    Eigen::Vector3d A_p_RF; /**< Position of the right foot contact frame in the inertial frame*/
    Eigen::Vector3d b_acc; /**< Bias of the accelerometer expressed in the IMU frame */
    Eigen::Vector3d b_gyro; /**< Bias of the gyroscope expressed in the IMU frame */
};

/**
* @brief Struct holding the elements of the state representation
*
*/
struct FBEOutput
{
    FBEInternalState x; /**< Current state estimate of the estimator */
    Eigen::MatrixXd P; /**< Current state covariance matrix */
};

/**
* @brief Struct holding the elements of the state representation
*
*/
struct FBEMeasurements
{
    Eigen::Vector3d acc, gyro; /**< accelerometer and gyroscope measurements expressed in IMU frame */
    Eigen::VectorXd encoders, encoders_speed; /**< Joint position and joint velocity measurements */
    bool lf_contact{false}; /**< left foot contact state */
    bool rf_contact{false}; /**< right foot contact state */
};

} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_IO_H

