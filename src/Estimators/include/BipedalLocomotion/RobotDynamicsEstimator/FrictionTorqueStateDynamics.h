/**
 * @file FrictionTorqueStateDynamics.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_FRICTION_TORQUE_STATE_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_FRICTION_TORQUE_STATE_DYNAMICS_H

#include <memory>
#include <BipedalLocomotion/RobotDynamicsEstimator/Dynamics.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelDynamics.h>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace RobotDynamicsEstimator
{

/**
 * The FrictionTorqueDynamics class is a concrete implementation of the Dynamics.
 * Please use this element if you want to use a specific friction torque model dynamics.
 * The FrictionTorqueDynamics represents the following equation in the continuous time:
 * \f[
 * \dot{\tau_{F}} = \ddot{s} ( k_{2} + k_{0} k_{1} (1 - tanh^{2} (k_{1} \dot{s})) )
 * \f]
 * since the friction model implemented by this class is defined as:
 * \f[
 * \tau_{F} = k_{0} tanh(k_{1} \dot{s}) + k_{2} \dot{s}
 * \f]
 * In the discrete time the dynamics is defined as:
 * \f[
 * \tau_{F,k+1} = \tau_{F,k} + \Delta T ( k_{2} + k_{0} k_{1} (1 - tanh^{2} (k_{1} \dot{s,k})) ) \ddot{s}
 * \f]
 */

class FrictionTorqueStateDynamics : public Dynamics
{
    Eigen::VectorXd m_jointVelocityFullModel; /**< Vector of joint velocities. */
    Eigen::VectorXd m_currentFrictionTorque; /**< Vector of friction torques. */
    Eigen::VectorXd m_k0, m_k1, m_k2; /**< Friction parameters (see class description). */
    double m_dT; /**< Sampling time. */
    int m_nrOfSubDynamics; /**< Number of sub-dynamics which corresponds to the number of sub-models. */
    std::vector<std::unique_ptr<SubModelDynamics>> m_subDynamics; /**< Vector of SubModelInversDynamics objects. */
    Eigen::VectorXd m_motorTorqueFullModel; /**< Motor torque vector of full-model. */
    Eigen::VectorXd m_frictionTorqueFullModel; /**< Friction torque vector of full-model. */
    std::vector<Eigen::VectorXd> m_subModelUpdatedJointAcceleration; /**< Updated joint acceleration of each sub-model. */
    Eigen::VectorXd m_jointAccelerationFullModel; /**< Vector of joint accelerations. */
    bool m_isSubModelListSet{false}; /**< Boolean flag saying if the sub-model list has been set. */

protected:
    Eigen::VectorXd m_tanhArgument;
    Eigen::VectorXd m_tanh;
    Eigen::VectorXd m_k0k1;
    Eigen::VectorXd m_argParenthesis;
    Eigen::VectorXd m_dotTauF;

public:
    /*
     * Constructor
     */
    FrictionTorqueStateDynamics();

    /*
     * Destructor
     */
    virtual ~FrictionTorqueStateDynamics();

    /**
     * Initialize the state dynamics.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                                       | Mandatory |
     * |:----------------------------------:|:--------:|:-------------------------------------------------------------------------------------------------------:|:---------:|
     * |               `name`               | `string` |   Name of the state contained in the `VariablesHandler` describing the state associated to this dynamics|    Yes    |
     * |            `covariance`            | `vector` |                                Process covariances                                                      |    Yes    |
     * |           `dynamic_model`          | `string` |               Type of dynamic model describing the state dynamics.                                      |    Yes    |
     * |             `elements`             | `vector` |  Vector of strings describing the list of sub variables composing the state associated to this dynamics.|    No     |
     * |            `friction_k0`           | `vector` |  Vector of coefficients k0. For more info check the class description.                                  |    Yes    |
     * |            `friction_k1`           | `vector` |  Vector of coefficients k1. For more info check the class description.                                  |    Yes    |
     * |            `friction_k2`           | `vector` |  Vector of coefficients k2. For more info check the class description.                                  |    Yes    |
     * |                `dT`                | `double` |                                Sampling time.                                                           |    Yes    |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;

    /**
     * Finalize the Dynamics.
     * @param stateVariableHandler object describing the variables in the state vector.
     * @note You should call this method after you add ALL the state dynamics to the state variable handler.
     * @return true in case of success, false otherwise.
     */
    bool finalize(const System::VariablesHandler& stateVariableHandler) override;

    /**
     * Set the SubModelKinDynWrapper object.
     * @param kinDynWrapper pointer to a SubModelKinDynWrapper object.
     * @return True in case of success, false otherwise.
     */
    bool setSubModels(const std::vector<SubModel>& subModelList, const std::vector<std::shared_ptr<SubModelKinDynWrapper>>& kinDynWrapperList) override;

    /**
      * Controls whether the variable handler contains the variables on which the dynamics depend.
      * @return True in case all the dependencies are contained in the variable handler, false otherwise.
      */
    bool checkStateVariableHandler() override;

    /**
     * Update the content of the element.
     * @return True in case of success, false otherwise.
     */
    bool update() override;

    /**
     * Set the state of the ukf needed to update the dynamics.
     * @param ukfState reference to the ukf state.
     */
     void setState(const Eigen::Ref<const Eigen::VectorXd> ukfState) override;

     /**
      * Set a `UKFInput` object.
      * @param ukfInput reference to the UKFInput struct.
      */
      void setInput(const UKFInput & ukfInput) override;
};

BLF_REGISTER_DYNAMICS(FrictionTorqueStateDynamics, ::BipedalLocomotion::Estimators::RobotDynamicsEstimator::Dynamics);

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_FRICTION_TORQUE_STATE_DYNAMICS_H
