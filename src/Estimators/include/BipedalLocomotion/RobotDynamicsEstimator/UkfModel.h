/**
 * @file UkfModel.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_UKF_MODEL_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_UKF_MODEL_H

#include <Eigen/Dense>
#include <memory>
#include <string>

// BLF
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/Dynamics.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/KinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace RobotDynamicsEstimator
{

/**
 * UkfModel is a base class to define the process and measurement model of the UKF.
 * It includes all the methods shared between the two concrete classes, as the
 * update of the robot dynamics and the robot state.
 */
class UkfModel
{
protected:
    bool m_isInitialized{false};
    bool m_isFinalized{false};
    Eigen::Vector3d m_gravity{0, 0, -Math::StandardAccelerationOfGravitation}; /**< Gravity vector. */
    double m_dT; /**< Sampling time */
    std::vector<std::pair<std::string, std::shared_ptr<Dynamics>>> m_dynamicsList; /**< List of the
                                                                                    dynamics
                                                                                    composing the
                                                                                    model.
                                                                                  */
    System::VariablesHandler m_stateVariableHandler; /**< Variable handler describing the state
                                                      vector. */
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynFullModel; /**< KinDynComputation object for
                                                                      the full model. */
    std::vector<SubModel> m_subModelList; /**< List of SubModel object describing the sub-models
                                           composing the full model. */
    std::vector<std::shared_ptr<KinDynWrapper>> m_kinDynWrapperList; /**< List of
                                                                              KinDynWrapper
                                                                              objects containing
                                                                              kinematic and dynamic
                                                                              information specific
                                                                              of each sub-model. */
    std::shared_ptr<const UkfInputProvider> m_ukfInputProvider; /**< Provider containing the updated
                                                                 robot state. */
    UKFInput m_ukfInput; /**< Struct containing the inputs for the ukf populated by the
                          ukfInputProvider. */
    Eigen::VectorXd m_jointVelocityState; /**< Joint velocity computed by the ukf. */
    Eigen::VectorXd m_jointAccelerationState; /**< Joint acceleration computed from forward dynamics
                                               which depends on the current ukf state. */
    Eigen::VectorXd m_currentState; /**< State estimated in the previous step. */
    std::vector<Eigen::VectorXd> m_subModelJointPos; /**< List of sub-model joint velocities. */
    std::vector<Eigen::VectorXd> m_subModelJointVel; /**< List of sub-model joint velocities. */
    std::vector<Eigen::VectorXd> m_subModelJointAcc; /**< List of sub-model joint accelerations. */
    std::vector<Eigen::VectorXd> m_subModelNuDot; /**< List of sub-model accelerations (base + joints
                                                   = nudot). */
    std::vector<Eigen::VectorXd> m_subModelJointMotorTorque; /**< List of sub-model joint motor
                                                              torques. */
    std::vector<Eigen::VectorXd> m_subModelFrictionTorque; /**< List of sub-model friction torques. */
    std::map<std::string, Math::Wrenchd> m_FTMap; /**< The map contains names of the ft sensors and
                                                   values of the wrench */
    std::map<std::string, Math::Wrenchd> m_extContactMap; /**< The map contains names of the ft
                                                           sensors and values of the wrench */
    std::map<std::string, Eigen::Vector3d> m_accMap; /**< The map contains names of the accelerometer sensors and
                                                   values of the linear accelerations */
    std::map<std::string, Eigen::Vector3d> m_gyroMap; /**< The map contains names of the gyroscope sensors and
                                                    values of the angular velocities */
    std::vector<Eigen::VectorXd> m_totalTorqueFromContacts; /**< Joint torques due to known and
                                                             unknown contacts on the sub-model. */
    Math::Wrenchd m_wrench; /**< Joint torques due to a specific contact. */
    Eigen::VectorXd m_measurement; /**< Measurements coming from the sensors. */
    std::map<std::string, Eigen::VectorXd> m_measurementMap; /**< Measurement map <measurement name,
                                                              measurement value>. */
    manif::SE3d::Tangent m_subModelBaseVelTemp; /**< Velocity of the base of the sub-model. */
    std::vector<Eigen::MatrixXd> m_tempJacobianList; /**< List of jacobians per eache submodel. */
    manif::SE3d m_worldTBase; /**< Sub-model base pose wrt the inertial frame */
    int m_offsetMeasurement; /**< Offset used to fill the measurement vector. */
    std::map<std::string, std::string> m_stateToUkfNames; /**< Map used to retrieve the name of the variable passed as state and the ukf name. */
    std::map<std::string, std::string> m_ukfNamesToMeasures; /**< Map used to retrieve the name of the variable passed as input and the ukf name. */
    Eigen::Vector3d m_sensorLinearAcceleration; /**< Linear acceleration measured by an accelerometer. */
    Eigen::Vector3d m_bOmegaIB; /**< Angular velocity of a frame. */
    manif::SE3Tangentd m_tempAccelerometerVelocity; /**< Velocity of an accelerometer. */
    manif::SE3Tangentd m_baseVelocity; /**< Submodel base velocity. */
    manif::SE3Tangentd m_baseAcceleration; /**< Submodel base acceleration. */

public:
    /**
     * @brief unpackState splits the state vector in all the variables composing the state.
     */
    void unpackState();

    /**
     * @brief updateState updates the robot state and dynamics and of the sub-model states and
     * dynamics by using the estimation of the previous step.
     * @return true in case of success, false otherwise.
     */
    bool updateState();
};

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_UKF_MODEL_H
