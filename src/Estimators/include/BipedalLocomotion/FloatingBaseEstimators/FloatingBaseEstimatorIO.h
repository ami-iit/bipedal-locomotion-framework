/**
 * @file FloatingBaseEstimatorIO.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_IO_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_IO_H

#include <Eigen/Dense>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <map>

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
    Eigen::Quaterniond imuOrientation; /**< Orientation of the base link IMU in the inertial frame (as a quaternion w,x,y,z) */
    Eigen::Vector3d imuPosition;   /**< Position of the base link IMU in the inertial frame */
    Eigen::Vector3d imuLinearVelocity; /**< linear part of the mixed-velocity representation of the IMU with respect to the inertial frame */
    Eigen::Quaterniond lContactFrameOrientation; /**< Orientation of the left foot contact frame in the inertial frame (as a quaternion w,x,y,z) */
    Eigen::Vector3d lContactFramePosition; /**< Position of the left foot contact frame in the inertial frame */
    Eigen::Quaterniond rContactFrameOrientation; /**< Orientation of the right foot contact frame in the inertial frame (as a quaternion w,x,y,z) */
    Eigen::Vector3d rContactFramePosition; /**< Position of the right foot contact frame in the inertial frame*/
    Eigen::Vector3d accelerometerBias; /**< Bias of the accelerometer expressed in the IMU frame */
    Eigen::Vector3d gyroscopeBias; /**< Bias of the gyroscope expressed in the IMU frame */

    Eigen::Vector3d imuAngularVelocity; /**< angular velocity of the IMU with respect to the inertial frame expressed in inertial frame, typically unused for strap-down IMU based EKF implementations*/
    std::map<int, BipedalLocomotion::Contacts::EstimatedContact> supportFrameData; /**< contact measurements */
};

/**
* @brief Struct holding the elements of the state representation
*
*/
struct Output
{
    InternalState state; /**< Current state estimate of the estimator */
    StateStdDev stateStdDev; /**< Current state covariance matrix */

    iDynTree::Transform basePose; /**< Estimated base link pose */
    iDynTree::Twist baseTwist; /**< Estimate base link velocity in mixed-velocity representation */
};

/**
* @brief Struct holding the elements of the state representation
*
*/
struct Measurements
{
    Eigen::Vector3d acc, gyro; /**< accelerometer and gyroscope measurements expressed in IMU frame */
    Eigen::VectorXd encoders, encodersSpeed; /**< Joint position and joint velocity measurements */
    bool lfInContact{false}; /**< left foot contact state */
    bool rfInContact{false}; /**< right foot contact state */
    
    /** stamped contact status, 
     * the usage of this map must be in a way 
     * that every time an element is used, 
     * it must be erased from the map 
     */
    std::map<int, BipedalLocomotion::Contacts::EstimatedContact > stampedContactsStatus;
};

} // namespace FloatingBaseEstimators
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_IO_H

