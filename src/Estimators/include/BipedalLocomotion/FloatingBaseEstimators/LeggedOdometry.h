/**
 * @file LeggedOdometry.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_LEGGED_ODOMETRY_H
#define BIPEDAL_LOCOMOTION_LEGGED_ODOMETRY_H

#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>

namespace BipedalLocomotion
{
namespace Estimators
{
/**
 * Floating base estimation algorithm using only kinematic measurements
 * by assuming atleast one frame on the robot is in contact with the environment
 *
 * Configuration parameters,
 *
 * |Group| Parameter | Type| Required | Description|
 * |:---:|:---:|:---:|:---:|:---:|
 * |LeggedOdom| - | group | Yes| Configuration group for initializing the legged odometry block |
 * ||initial_fixed_frame|string|Yes| Frame on the URDF model of the robot assumed to be fixed (in contact) during the initialization phase|
 * ||initial_ref_frame_for_world|string| No| Frame on the URDF model of the robot to be used as a reference to initialize the inertial frame for the estimation. If not present, the initial fixed frame is assumed as the reference|
 * ||initial_world_orientation_in_ref_frame|vector of 4 doubles| No | Orientation of the inertial frame wrt the the reference frame as quaternion wxyz. If not present, assumed to be unit quaternion.|
 * ||initial_world_position_in_ref_frame| vector of 3 doubles | No | Position of the inertial frame wrt the reference frame as xyz. If not present,assumed to be zero position|
 * ||switching_pattern|string|No| Options: latest, lastActive, useExternal. Switching pattern to decide the fixed frame from the set of all frames in contact. latest chooses the recently switched contact frame, lastActive chooses the earliest switched contact frame. Default is latest|
 * ||vel_computation_method| string | No| Options: single, multiAvg, multiLS, multLSJvel. Method used for computing the floating base velocity using the fixed frame constraints. single uses only the fixed frame, multiAvg computes a simple average from all the contact frames, multLS uses a least square solution and multLSJVel regualrizes the least square solution by augmenting joint velocities in the computation. Default is multiLS.|
 * ||wLin|double|No| weight used for linear velocity in the least square computation of base velocity. If not present, set to 1.0|
 * ||wAng|double|No| weight used for angular velocity in the least square computation of base velocity. If not present, set to 0.5|
 * ||wJVel|double|No| weight used for linear velocity in the least square (with joint velocities) computation of base velocity. If not present, set to 10.0|
 * ||reg|double|No| regularization used for matrix inversion operation in least square. If not present, set to 1e-6|
 */
class LeggedOdometry : public FloatingBaseEstimator
{
public:
    /**
     * Constructor
     */
    LeggedOdometry();

    /**
     * Destructor (necessary for PIMPL idiom)
     */
    ~LeggedOdometry();

    /**
     * To prevent function hiding due to overloading of virtual methods
     */
    using FloatingBaseEstimator::resetEstimator;

    /**
     * Reset the internal state of the estimator using the initialized parameters
     * @return True in case of success, false otherwise.
     *
     * @note reset and advance estimator to get updated estimator output
     */
    bool resetEstimator();

    /**
     * Reset the internal state of the estimator by setting a new reference for the inertial frame
     * @param[in] refFramForWorld  frame from the model as a reference to set the new inertial frame
     * @param[in] worldOrientationInRefFrame  orientation of the inertial frame wrt to the reference frame
     * @param[in] worldPositionInRefFrame  position of the inertial frame wrt to the reference frame
     * @return True in case of success, false otherwise.
     *
     * @note reset and advance estimator to get updated estimator output
     */
    bool resetEstimator(const std::string& refFramForWorld,
                        const Eigen::Quaterniond& worldOrientationInRefFrame,
                        const Eigen::Vector3d& worldPositionInRefFrame);

    /**
     * Change fixed frame externally
     * @param[in] frameIndex  valid frame index from the model
     */
    bool changeFixedFrame(const std::ptrdiff_t& frameIndex);

    /**
     * Change fixed frame externally
     * @param[in] frameIndex  valid frame name from the model
     */
    bool changeFixedFrame(const std::string& frameName);

    /**
     * Change fixed frame externally by mentioning the world_H_frame transform
     * @param[in] frameIndex  valid frame from the model
     * @param[in] frameOrientationInWorld  orientation of the fixed frame wrt the inertial frame
     * @param[in] framePositionInWorld  position of the fixed frame wrt the inertial frame
     * @note ensure to pass unit quaternion
     */
    bool changeFixedFrame(const std::ptrdiff_t& frameIndex,
                          const Eigen::Vector4d& frameOrientationInWorld,
                          const Eigen::Vector3d& framePositionInWorld);

    /**
     * Get the index of the frame currently in contact used for the legged odometry computations
     * @return Index of the frame in contact, invalid frame index if no contact.
     */
    int getFixedFrameIdx();

    /**
     * Get the psoe of the current fixed frame in the inertial frame
     * @return const ref of pose of the fixed frame
     */
    manif::SE3d& getFixedFramePose() const;

protected:
    /**
    * These custom parameter specifications should be specified by the derived class.
    * @param[in] handler configure the custom parameters for the estimator
    * @return bool
    */
    virtual bool customInitialization(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler) override;

    /**
    * Update the base state estimate using kinematics and contact measurements
    * @param[in] meas measurements to update states
    * @param[in] dt sampling period in seconds
    * @return True in case of success, false otherwise.
    */
    virtual bool updateKinematics(FloatingBaseEstimators::Measurements& meas,
                                  const double& dt) override;


private:
    /**
    * Private implementation of the class
    */
    class Impl;
    std::unique_ptr<Impl> m_pimpl; /**< Pointer to implementation */
};

} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_INVEKF_BASE_ESTIMATOR_H
