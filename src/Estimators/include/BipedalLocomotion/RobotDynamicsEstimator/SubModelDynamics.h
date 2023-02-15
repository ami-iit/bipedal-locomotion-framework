/**
 * @file SubModelDynamics.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SUB_MODEL_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_SUB_MODEL_DYNAMICS_H

#include <memory>
#include <string>
#include <map>

#include <Eigen/Dense>

#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelKinDynWrapper.h>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace RobotDynamicsEstimator
{

struct SubModelDynamics
{
    std::shared_ptr<SubModelKinDynWrapper> kinDynWrapper; /**< object containing the kinematics and dynamics properties of a sub-model. */
    SubModel subModel; /**< Sub-model object storing information about the model (joints, sensors, applied wrenches) */
    std::map<std::string, Math::Wrenchd> FTMap; /**< The map contains names of the ft sensors and values of the wrench */
    std::map<std::string, Math::Wrenchd> extContactMap; /**< The map contains names of the ft sensors and values of the wrench */
    manif::SE3d::Tangent baseAcceleration; /**< Velocity of the base of the sub-model. */
    Eigen::VectorXd motorTorque; /**< Motor torque vector of sub-model. */
    Eigen::VectorXd frictionTorque; /**< Friction torque vector of sub-model. */
    Eigen::VectorXd jointVelocity; /**< Joint velocities of sub-model. */
    Eigen::VectorXd totalTorqueFromContacts; /**< Joint torques due to known and unknown contacts on the sub-model. */
    Math::Wrenchd wrench; /**< Joint torques due to a specific contact. */
    Eigen::VectorXd torqueFromContact; /**< Joint torques due to a specific contact. */

    /*
     * Constructor
     */
    SubModelDynamics();

    /*
     * Destructor
     */
    virtual ~SubModelDynamics();

    /**
     * @brief Set the SubModelKinDynWrapper object.
     * @param kinDynWrapper pointer to a SubModelKinDynWrapper object.
     * @return true in case of success, false otherwise.
     */
    bool setKinDynWrapper(std::shared_ptr<SubModelKinDynWrapper> kinDynWrapper);

    /**
     * @brief Set the SubModel object.
     * @param subModel pointer to a SubModel object.
     * @return true in case of success, false otherwise.
     */
    bool setSubModel(const SubModel& subModel);

    /**
     * @brief Initialize the SubDynamics.
     * @return true in case of success, false otherwise.
     */
    bool initialize();

    /**
     * @brief setState set the state of the variables used to compute the inverse dynamics.
     * @param ukfState an Eigen::Vector representing the state computed by the ukf estimator.
     * @param jointVelocityFullModel `Eigen::Vector` representing the velocity of the robot joints.
     * @param motorTorqueFullModel `Eigen::Vector` representing the torque of the robot motors.
     * @param frictionTorqueFullModel `Eigen::Vector` representing the friction torque at the robot motors.
     * @param variableHandler handler of the state of the ukf.
     */
    void setState(const Eigen::Ref<const Eigen::VectorXd> ukfState,
                  const Eigen::Ref<const Eigen::VectorXd> jointVelocityFullModel,
                  const Eigen::Ref<const Eigen::VectorXd> motorTorqueFullModel,
                  const Eigen::Ref<const Eigen::VectorXd> frictionTorqueFullModel,
                  const System::VariablesHandler& variableHandler);

    /**
     * @brief update computes the joint accelerations given by the forward dynamics and the joint velocities given
     * by the numerical integration of the joint accelerations.
     * @param fullModelBaseAcceleration is a `manif::SE3d::Tangent` representing the acceleration of the base of the full model.
     * @param fullModelJointAcceleration is a `Eigen::VectorXd` representing the acceleration of the joints of the full model.
     * @param updatedJointAcceleration is a `Eigen::VectorXd` representing the joint accelerations of the sub-model.
     * @return true in case of success, false otherwise.
     * @note this method computes the joint acceleration of the sub-model, but it requires the joint acceleration of the full model.
     * Actually the only joint accelerations needed to compute the ones of this sub-model are the joint accelerations
     * of the previous sub-models as they are used to compute the acceleration of the base of this sub-model. The rest of the joint
     * accelerations, which are computed here, can be set to zero as those values are not used.
     */
    bool update(manif::SE3d::Tangent& fullModelBaseAcceleration,
                Eigen::Ref<const Eigen::VectorXd> fullModelJointAcceleration,
                Eigen::Ref<Eigen::VectorXd> updatedJointAcceleration);

    /**
     * @brief Compute the contribution of external contacts on the joint torques.
     */
    void computeTotalTorqueFromContacts();
};

} // RobotDynamicsEstimator
} // Estimators
} // BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SUB_MODEL_DYNAMICS_H
