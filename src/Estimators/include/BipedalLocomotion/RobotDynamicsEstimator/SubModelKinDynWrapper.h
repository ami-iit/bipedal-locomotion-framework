/**
 * @file SubModelKinDynWrapper.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_SUB_MODEL_KINDYN_WRAPPER_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_SUB_MODEL_KINDYN_WRAPPER_H

#include <unordered_map>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/Model.h>

// Eigen
#include <Eigen/Dense>

// RDE
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace RobotDynamicsEstimator
{

/**
 * The enum class UpdateMode says if the updateState method should update only the
 * information related to the robot dynamics used to compute the forward dynamics
 * (i.e. mass matrix, generalized forces, KinDynComputation objects, jacobians
 * associated to force/torque sensors and external contacts) or also additional
 * objects like jacobiand associated to accelerometers, gyroscopes,
 * bias accelerations, rotation matrices of the sensors, robot base velocity,
 * robot base acceleration.
 */
enum class UpdateMode {
    Full,
    RobotDynamicsOnly
};

/**
 * SubModelKinDynWrapper is a concrete class and implements a wrapper of the KinDynComputation class
 * from iDynTree. The class is used to take updated the sub-model kinematics and dynamics
 */
class SubModelKinDynWrapper
{
    SubModel m_subModel; /**< SubModel struct containing all the information about the sub-model */
    iDynTree::KinDynComputations m_kinDyn; /**< kindyncomputation object used to get the sub-model
                                            kinematic and dynamic information */
    Eigen::MatrixXd m_massMatrix; /**< Mass matrix of the sub-model */
    Eigen::VectorXd m_genForces; /**< Generalized force vector */
    std::unordered_map<std::string, Eigen::MatrixXd> m_jacFTList; /**< Jacobians of the FT sensors */
    std::unordered_map<std::string, Eigen::MatrixXd> m_jacAccList; /**< Jacobians of the accelerometer
                                                                  sensors */
    std::unordered_map<std::string, Eigen::MatrixXd> m_jacGyroList; /**< Jacobians of the gyroscope
                                                                   sensors */
    std::unordered_map<std::string, Eigen::MatrixXd> m_jacContactList; /**< Jacobians of the external
                                                                      contacts */
    std::unordered_map<std::string, Eigen::VectorXd> m_dJnuList; /**< Accelerometer bias accelerations
                                                              */
    std::unordered_map<std::string, manif::SO3d> m_accRworldList; /**< Rotation matrix of the
                                                                 accelerometer frame wrt world */
    std::unordered_map<std::string, manif::SE3d::Tangent> m_accVelList; /**< Acceleration of the
                                                                 accelerometers */
    manif::SE3d::Tangent m_baseVelocity; /**< Velocity of the base of the sub-model */
    std::string m_baseFrame; /**< Name of the base frame of the sub-model */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynFullModel;

    manif::SE3d::Tangent m_subModelBaseAcceleration; /** Acceleration of the sub-model base. */

protected:
    int m_numOfJoints; /**< Number of joints in the sub-model */
    Eigen::MatrixXd m_FTranspose; /**< It is the bottom-left block of the mass matrix, that is,
                                     massmatrix[6:end, 0:6] */
    Eigen::MatrixXd m_H; /**< It is the bottom-right block of the mass matrix, that is,
                            massmatrix[6:end, 6:end] */
    Eigen::VectorXd m_genBiasJointTorques; /**< Generalized bias joint torques */
    Eigen::MatrixXd m_pseudoInverseH; /**< m_H pseudoinverse */
    Eigen::VectorXd m_FTBaseAcc; /**< FTranspose times base acceleration */
    Eigen::VectorXd m_totalTorques; /**< total torques acting on the joints, before being converted
                                       into joint acceleration through the mass matrix */
    Eigen::VectorXd m_jointPositionModel; /**< Vector of joint positions from full model */
    Eigen::VectorXd m_jointVelocityModel; /**< Vector of joint velocities from full model */
    Eigen::Vector3d m_worldGravity; /**< world gravity acceleration */
    manif::SE3d m_worldTBase; /**< Pose of base of the sub-model wrt the world frame */
    Eigen::VectorXd m_qj; /**< Vector of joint positions of the sub-model */
    Eigen::VectorXd m_dqj; /**< Vector of joint velocities of the sub-model */

protected:
    /**
     * updateDynamicsVariableState updates the value of all the member variables containing
     * information about the robot kinematics and dynamics
     * @param updateMode says if update only objects for computing the forward dynamics or also information
     * associated to additional sensors like accelerometers/gyroscopes
     */
    bool updateDynamicsVariableState(UpdateMode updateMode);

    /**
     * @brief Compute the contribution of external contacts on the joint torques.
     */
    void computeTotalTorqueFromContacts();

public:
    /**
     * @brief set kinDyn
     * @param kinDyn is an iDynTree KinDynComputation object handling the kinematics and the
     * dynamics of the model.
     * @return a boolean value saying if the input pointer is valid
     */
    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    /**
     * @brief initialize initializes the kindyncomputation object and resizes
     * all the variable not allocated yet.
     * @param subModel a struct of type SubModif el describing the subModel structure and model.
     * @return a boolean value saying if all the variables are initialized correctly.
     */
    bool
    initialize(const SubModel& subModel);

    /**
     * @brief updateState updates the state of the KinDynWrapper object.
     * @param robotBaseAcceleration is a manif::SE3d::Tangent representing the robot base acceleration.
     * @param robotJointAcceleration is a Eigen reference to a Eigen::VectorXd containing the joint accelerations.
     * @param updateMode says if update only objects for computing the forward dynamics or also information
     * associated to additional sensors like accelerometers/gyroscopes
     * @return a boolean value saying if the subModelList has been created correctly.
     */
    bool updateState(const manif::SE3d::Tangent& robotBaseAcceleration,
                     Eigen::Ref<const Eigen::VectorXd> robotJointAcceleration,
                     UpdateMode updateMode);

    /**
     * @brief forwardDynamics computes the free floaing forward dynamics
     * @param motorTorqueAfterGearbox a vector of size number of joints containing the motor torques
     * times the gearbox ratio.
     * @param frictionTorques a vector of size number of joints containing the friction torques.
     * @param tauExt a vector of size number of joints containing the joint torques generated by the
     * external contacts.
     * @param baseAcceleration the sub-model base acceleration.
     * @return a boolean value
     */
    bool forwardDynamics(Eigen::Ref<Eigen::VectorXd> motorTorqueAfterGearbox,
                         Eigen::Ref<Eigen::VectorXd> frictionTorques,
                         Eigen::Ref<Eigen::VectorXd> tauExt,
                         Eigen::Ref<Eigen::VectorXd> baseAcceleration,
                         Eigen::Ref<Eigen::VectorXd> jointAcceleration);

    /**
     * @brief getBaseAcceleration gets the acceleration of the sub-model base.
     * @return subModelBaseAcceleration the acceleration of the sub-model base.
     */
    const manif::SE3d::Tangent& getBaseAcceleration();

    /**
     * @brief getBaseVelocity gets the acceleration of the sub-model base.
     * @return baseVelocity the velocity of the sub-model base.
     */
    const manif::SE3d::Tangent&
    getBaseVelocity();

    /**
     * Getters
     */

    /** Access the name of the sub-model base frame.
     * @return The name of the base frame.
     */
    const std::string& getBaseFrameName() const;

    /** Access massMatrix
     * @return The current value of massMatrix.
     */
    const Eigen::Ref<const Eigen::MatrixXd> getMassMatrix() const;

    /** Access genForces
     * @return The current value of genForces.
     */
    const Eigen::Ref<const Eigen::VectorXd> getGeneralizedForces() const;

    /** Access the jacobian of the force/torque sensor specified by ftName.
     * @throws an out_of_range exception if ftName is not in the list of force/torque sensors of the
     * sub-model.
     * @return a boolean value saying if the desired jacobian is found.
     */
    const Eigen::Ref<const Eigen::MatrixXd> getFTJacobian(const std::string& ftName) const;

    /** Access the jacobian of the accelerometer sensor specified by accName.
     * @throws an out_of_range exception if accName is not in the list of accelerometer sensors of
     * the sub-model.
     * @return a boolean value saying if the desired jacobian is found.
     */
    const Eigen::Ref<const Eigen::MatrixXd>
    getAccelerometerJacobian(const std::string& accName) const;

    /** Access the jacobian of the gyroscope sensor specified by gyroName.
     * @throws an out_of_range exception if gyroName is not in the list of gyroscope sensors of the
     * sub-model.
     * @return a boolean value saying if the desired jacobian is found.
     */
    const Eigen::Ref<const Eigen::MatrixXd> getGyroscopeJacobian(const std::string& gyroName) const;

    /** Access the jacobian of the contact frame specified by extContactName.
     * @throws an out_of_range exception if extContactName is not in the list of external contacts
     * of the sub-model.
     * @return a boolean value saying if the desired jacobian is found.
     */
    const Eigen::Ref<const Eigen::MatrixXd>
    getExtContactJacobian(const std::string& extContactName) const;

    /** Access the bias acceleration referred to the accelerometer specified by accName.
     * @throws an out_of_range exception if accName is not in the list of accelerometer sensors of
     * the sub-model.
     * @return a boolean value saying if the bias acceleration is found.
     */
    const Eigen::Ref<const Eigen::VectorXd>
    getAccelerometerBiasAcceleration(const std::string& accName) const;

    /** Access the rotation matrix between the senosr accName and the world.
     * @throws an out_of_range exception if accName is not in the list of accelerometer sensors of
     * the sub-model.
     * @return a boolean value saying if the rotation matrix is found.
     */
    const manif::SO3d& getAccelerometerRotation(const std::string& accName) const;

    /**
     * @brief getAccelerometerVelocity access the velocity of the accelerometer specified by the input param.
     * @param accName is the name of the accelerometer.
     * @return the velocity of the accelerometer.
     */
    const manif::SE3d::Tangent& getAccelerometerVelocity(const std::string& accName);
};

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_SUB_MODEL_KINDYN_WRAPPER_H
