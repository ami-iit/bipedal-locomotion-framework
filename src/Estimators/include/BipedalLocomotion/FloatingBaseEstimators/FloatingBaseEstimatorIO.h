/**
 * @file FloatingBaseEstimatorIO.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_FLOATING_BASE_ESTIMATOR_IO_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_FLOATING_BASE_ESTIMATOR_IO_H

#include <Eigen/Dense>
#include <manif/manif.h>

#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimatorParams.h>
#include <BipedalLocomotion/Contacts/Contact.h>
#include <manif/manif.h>

#include <unordered_map>
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
    std::map<int, BipedalLocomotion::Contacts::EstimatedContact> supportFrameData; /**< estimated contacts */
    std::map<int, BipedalLocomotion::Contacts::EstimatedLandmark> landmarkData; /**< estimated landmarks */
};

/**
* @brief Struct holding the elements of the estimator output
*
*/
struct Output
{
    InternalState state; /**< Current state estimate of the estimator */
    StateStdDev stateStdDev; /**< Current state covariance matrix */

    manif::SE3d basePose; /**< Estimated base link pose */
    Eigen::Matrix<double, 6, 1> baseTwist; /**< Estimate base link velocity in mixed-velocity representation */
};

/**
* @brief Struct holding the elements of the measurements data
*
*/
struct Measurements
{
    using ContactWrench_t = Eigen::Matrix<double, 6, 1>;

    double ts; /**< receive timestamp */
    Eigen::Vector3d acc, gyro; /**< accelerometer and gyroscope measurements expressed in IMU frame */
    Eigen::VectorXd encoders, encodersSpeed; /**< Joint position and joint velocity measurements */
    bool lfInContact{false}; /**< left foot contact state */
    bool rfInContact{false}; /**< right foot contact state */
    std::string lfContactFrameName{"l_sole"}, rfContactFrameName{"r_sole"}; /**< Required for InvEKF */
    std::unordered_map<std::string, ContactWrench_t> contactWrenches; /**< contact wrenches identified by contact frame names */

    /** stamped relative poses,
    *
    * @note at the implementation level of FloatingBaseEstimator,
    * the usage of this map is in a way
    * that at every iteration, at the end of the
    * advance() call of the estimator, this map is cleared
    * so as to not use delayed or outdated measurments
    *
    * @note we maintain a map (sorted elements) in order to
    * maintain accessibility of incrementally augmented variables
    * from the state and the covariance matrix
    */
    std::map<int, BipedalLocomotion::Contacts::EstimatedContact > stampedRelLandmarkPoses;

    /** stamped contact status,
    *
    * @note at the implementation level of FloatingBaseEstimator,
    * the usage of this map is in a way
    * that at every iteration, at the end of the
    * advance() call of the estimator, this map is cleared
    * so as to not use delayed or outdated measurments
    *
    * @note we maintain a map (sorted elements) in order to
    * maintain accessibility of incrementally augmented variables
    * from the state and the covariance matrix
    */
    std::map<int, BipedalLocomotion::Contacts::EstimatedContact > stampedContactsStatus;
};

} // namespace FloatingBaseEstimators
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_FLOATING_BASE_ESTIMATOR_IO_H
