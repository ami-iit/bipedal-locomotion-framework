/**
 * @file BaseEstimatorFromFootIMU.h
 * @authors Guglielmo Cervettini
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be
 * modified and distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_BASE_ESTIMATION_FROM_FOOT_IMU_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_BASE_ESTIMATION_FROM_FOOT_IMU_H

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model.h>
#include <manif/SE3.h>
#include <manif/SO3.h>

#include <memory>

namespace BipedalLocomotion
{
namespace Estimators
{

/**
 * BaseEstimatorFromFootIMUState contains the internal state of the estimator.
 */
struct BaseEstimatorFromFootIMUState
{
    manif::SE3d basePose; /**< final output of the estimator - pose of the robot
                             root link. */
};

/**
 * BaseEstimatorFromFootIMUInput contains the input of the BaseEstimatorFromFootIMU class.
 */
struct BaseEstimatorFromFootIMUInput
{
    Eigen::VectorXd jointPositions; /**< vector of the robot joint positions */
    Eigen::VectorXd jointVelocities; /**< vector of the robot joint velocities */
    manif::SE3d desiredFootPose; /**< desired orientation and position of the foot
                                  as per footstep planner output */
    manif::SO3d measuredRotation; /**< actual orientation of the foot measured by
                                   on-board IMU */
};

/**
 * BaseEstimatorFromFootIMU implements the propagation of the foot pose to the root link through the
 * kinematic chain given by the leg joints positions.
 *
 * This class assumes that the foot has a rectangular shape as shown in the following schematics
 *          +
 *    p2 __|__   p1   __
 *      |  |  |         |
 *   ___|__|__|___+     | FOOT
 *      |  |  |         | LENGTH
 *      |__|__|       __|
 *    p3   |     p4
 *
 *      |_____|
 *       FOOT
 *       WIDTH
 */
class BaseEstimatorFromFootIMU
    : public BipedalLocomotion::System::Advanceable<BaseEstimatorFromFootIMUInput,
                                                    BaseEstimatorFromFootIMUState>
{
public:
    /**
     * Set the iDynTree::Model of the robot to be used by the estimator.
     * @param model the model considered in the estimator
     * @return true in case of success, false otherwise.
     */
    bool setModel(const iDynTree::Model& model);

    // clang-format off
    /**
     * Initialize the BaseEstimatorFromFootIMU block.
     * The setModel and setStanceFootRF methods must be called before initialization.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are required
     * |   Parameter Name    |   Type   |                Description               | Mandatory |
     * |:-------------------:|:--------:|:----------------------------------------:|:---------:|
     * |  `foot_width_in_m`  | `double` | Transversal dimension of the robot foot  |    Yes    |
     * |  `foot_length_in_m` | `double` | Longitudinal dimension of the robot foot |    Yes    |
     * Moreover the user needs to add a group named `MODEL_INFO` that contains the following parameters.
     * | Parameter Name |     Type      |                                  Description                                          | Mandatory |
     * |:--------------:|:-------------:|:-------------------------------------------------------------------------------------:|:---------:|
     * |  `base_frame`  | `std::string` |                 Name of the frame whose pose must be estimated                        |    Yes    |
     * |  `foot_frame`  | `std::string` |  Name of the frame of one of the possible stance feet (whose orientation is measured) |    Yes    |
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;
    // clang-format on

    /**
     * Set the input of the estimator.
     * @param input the input of the system.
     * @return true in case of success, false otherwise.
     */
    bool setInput(const BaseEstimatorFromFootIMUInput& input) override;

    /**
     * Perform one step of the estimator.
     * The estimator must have been initialized first.
     * @return true in case of success, false otherwise.
     */
    bool advance() override;

    /**
     * Check if the output of the estimator is valid.
     * @return true in case of success, false otherwise.
     */
    bool isOutputValid() const override;

    /**
     * Get the internal state of the BaseEstimatorFromFootIMU.
     * @return a const reference to the state of the estimator.
     */
    const BaseEstimatorFromFootIMUState& getOutput() const override;

    /**
     * Set the state of the BaseEstimatorFromFootIMU.
     * @param state of the BaseEstimatorFromFootIMU
     * @note This method should be called only if the user want to force the state
     * of the system.
     */
    void setState(const BaseEstimatorFromFootIMUState& state);

private:
    BaseEstimatorFromFootIMUInput m_input; /**< Last input stored in the estimator */
    BaseEstimatorFromFootIMUState m_state; /**< Current state stored in the estimator */

    // Geometric quantities of the foot
    double m_footWidth; /**< Lateral dimension of the robot foot */
    double m_footLength; /**< Frontal dimension of the robot foot */

    /**
     * Define the 4 foot vertices in World reference frame:
     *
     *        +
     *  m_p2 __|__ m_p1   __
     *      |  |  |         |
     *   ___|__|__|___+     | FOOT
     *      |  |  |         | LENGTH
     *      |__|__|       __|
     *  m_p3   |   m_p4
     *
     *      |_____|
     *       FOOT
     *       WIDTH
     *
     */
    std::vector<Eigen::Vector3d> m_cornersInLocalFrame; /**< this implementation is considering
                                                           rectangular feet (4 corners) */
    std::vector<Eigen::Vector3d> m_transformedFootCorners; /**< vector of the foot corners
                                                              transformed into the inertial frame */
    Eigen::Vector3d m_desiredTranslation; /**< the position vector extracted from the
                                             `desiredFootPose` */
    Eigen::Matrix3d m_desiredRotation; /**< the rotation matrix extracted from the `desiredFootPose`
                                        */
    Eigen::Vector3d m_desiredRPY; /**< the rotation matrix extracted from the `desiredFootPose`
                                     converted into Euler angles */
    Eigen::Matrix3d m_measuredRotation; /**< the measured foot orientation casted manif::SO3d -->
                                           Eigen::Matrix3d */
    Eigen::Vector3d m_measuredRPY; /**< the measured foot orientation converted into Euler angles */
    manif::SO3d m_measuredRotationCorrected; /**< rotation matrix that employs: measured Roll,
                                                measured Pitch, desired Yaw */
    manif::SE3d m_measuredFootPose; /**< the final foot pose matrix obtained through measured and
                                       desired quantities */
    manif::SE3d m_frame_H_link; /**< coordinate change matrix from foot link frame to foot sole frame
                                 */
    Eigen::Vector3d m_gravity;
    iDynTree::KinDynComputations m_kinDyn;
    iDynTree::Model m_model;
    iDynTree::LinkIndex m_linkIndex;
    int m_frameIndex;
    std::string m_frameName;
    int m_baseFrame; /**< Index of the frame whose pose needs to be estimated */
    std::string m_footFrameName; /**< reference frames of the possible stance feet (whose
                                    orientations are measured)*/
    bool m_isOutputValid{false};
    bool m_isModelSet{false};
    bool m_isInitialized{false};
};

} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_BASE_ESTIMATION_FROM_FOOT_IMU_H
