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
namespace FloatingBaseEstimators
{
/**
* @brief Struct holding the elements of the state representation
*
*/
struct InternalState
{


    Eigen::Quaterniond imu_orientation; /**< Orientation of the base link IMU in the inertial frame (as a quaternion w,x,y,z) */
    Eigen::Vector3d imu_position;   /**< Position of the base link IMU in the inertial frame */
    Eigen::Vector3d imu_linear_velocity; /**< linear part of the mixed-velocity representation of the IMU with respect to the inertial frame */
    Eigen::Quaterniond l_contact_frame_orientation; /**< Orientation of the left foot contact frame in the inertial frame (as a quaternion w,x,y,z) */
    Eigen::Vector3d l_contact_frame_position; /**< Position of the left foot contact frame in the inertial frame */
    Eigen::Quaterniond r_contact_frame_orientation; /**< Orientation of the right foot contact frame in the inertial frame (as a quaternion w,x,y,z) */
    Eigen::Vector3d r_contact_frame_position; /**< Position of the right foot contact frame in the inertial frame*/
    Eigen::Vector3d accelerometer_bias; /**< Bias of the accelerometer expressed in the IMU frame */
    Eigen::Vector3d gyroscope_bias; /**< Bias of the gyroscope expressed in the IMU frame */
};

/**
* @brief Struct holding the elements of the state representation
*
*/
struct Output
{
    InternalState state; /**< Current state estimate of the estimator */
    Eigen::MatrixXd state_var; /**< Current state covariance matrix */
};

/**
* @brief Struct holding the elements of the state representation
*
*/
struct Measurements
{
    Eigen::Vector3d acc, gyro; /**< accelerometer and gyroscope measurements expressed in IMU frame */
    Eigen::VectorXd encoders, encoders_speed; /**< Joint position and joint velocity measurements */
    bool lf_in_contact{false}; /**< left foot contact state */
    bool rf_in_contact{false}; /**< right foot contact state */
};

} // namespace FloatingBaseEstimators
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_IO_H

