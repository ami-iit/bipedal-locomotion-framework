/**
 * @file FrictionTorqueStateDynamics.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_FRICTION_TORQUE_STATE_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_FRICTION_TORQUE_STATE_DYNAMICS_H

#include <BipedalLocomotion/RobotDynamicsEstimator/Dynamics.h>
#include <memory>

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
 * \tau_{F,k+1} = \tau_{F,k} + \Delta T \ddot{s} ( k_{2} + k_{0} k_{1} / cosh^{2}(k_{1} \dot{s,k}) )
 * \f]
 */
class FrictionTorqueStateDynamics : public Dynamics
{
    Eigen::VectorXd m_jointVelocityFullModel; /**< Vector of joint velocities. */
    Eigen::VectorXd m_Fc, m_Fs, m_Fv; /**< Friction parameters (see class description). */
    std::chrono::nanoseconds m_dT{std::chrono::nanoseconds::zero()}; /**< Sampling time. */
    Eigen::VectorXd m_frictionTorqueFullModel; /**< Friction torque vector of full-model. */
    UKFInput m_ukfInput;
    std::string m_name; /**< Name of dynamics. */
    std::vector<std::string> m_elements = {}; /**< Elements composing the variable vector. */
    System::VariablesHandler m_stateVariableHandler; /**< Variable handler describing the variables
                                                        and the sizes in the ukf state vector. */
    Eigen::VectorXd m_coshArgument;
    Eigen::VectorXd m_coshsquared;
    Eigen::VectorXd m_FcFs;
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
     * @param name name of the dynamics.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                                       | Mandatory |
     * |:----------------------------------:|:--------:|:-------------------------------------------------------------------------------------------------------:|:---------:|
     * |            `input_name`            | `string` |        Name of the variable in input to the main application corresponding to this dynamics             |    Yes    |
     * |            `covariance`            | `vector` |                                Process covariances                                                      |    Yes    |
     * |         `initial_covariance`       | `vector` |                             Initial state covariances                                                   |    Yes    |
     * |           `dynamic_model`          | `string` |               Type of dynamic model describing the state dynamics.                                      |    Yes    |
     * |            `friction_k0`           | `vector` |  Vector of coefficients k0. For more info check the class description.                                  |    Yes    |
     * |            `friction_k1`           | `vector` |  Vector of coefficients k1. For more info check the class description.                                  |    Yes    |
     * |            `friction_k2`           | `vector` |  Vector of coefficients k2. For more info check the class description.                                  |    Yes    |
     * |                `dT`                | `double` |                                Sampling time.                                                           |    Yes    |
     * @return True in case of success, false otherwise.
     */
    bool
    initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler, const std::string& name) override;

    /**
     * Finalize the Dynamics.
     * @param stateVariableHandler object describing the variables in the state vector.
     * @note You should call this method after you add ALL the state dynamics to the state variable
     * handler.
     * @return true in case of success, false otherwise.
     */
    bool finalize(const System::VariablesHandler& stateVariableHandler) override;

    /**
     * Set the KinDynWrapper object.
     * @param subModelList list of SubModel objects
     * @param kinDynWrapperList list of pointers to KinDynWrapper objects.
     * @return True in case of success, false otherwise.
     */
    bool setSubModels(
        const std::vector<SubModel>& subModelList,
        const std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList) override;

    /**
     * Controls whether the variable handler contains the variables on which the dynamics depend.
     * @return True in case all the dependencies are contained in the variable handler, false
     * otherwise.
     */
    bool checkStateVariableHandler() override;

    /**
     * Update the content of the element.
     * @return True in case of success, false otherwise.
     */
    bool update() override;

    /**
     * Set the state of the ukf needed to update the dynamics of the state variable associated to
     * ths object.
     * @param ukfState reference to the ukf state.
     */
    void setState(const Eigen::Ref<const Eigen::VectorXd> ukfState) override;

    /**
     * Set a `UKFInput` object.
     * @param ukfInput reference to the UKFInput struct.
     */
    void setInput(const UKFInput& ukfInput) override;
};

BLF_REGISTER_UKF_DYNAMICS(FrictionTorqueStateDynamics,
                          ::BipedalLocomotion::Estimators::RobotDynamicsEstimator::Dynamics);

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_FRICTION_TORQUE_STATE_DYNAMICS_H
