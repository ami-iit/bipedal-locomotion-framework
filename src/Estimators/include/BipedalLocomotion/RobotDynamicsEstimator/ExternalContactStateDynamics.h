/**
 * @file ExternalContactStateDynamics.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_EXTERNAL_CONTACT_STATE_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_EXTERNAL_CONTACT_STATE_DYNAMICS_H

#include <BipedalLocomotion/RobotDynamicsEstimator/Dynamics.h>
#include <memory>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace RobotDynamicsEstimator
{

/**
 * The ExternalContactStateDynamics class is a concrete implementation of the Dynamics.
 * Please use this element if you do not know the specific dynamics of a state variable.
 * The ExternalContactStateDynamics represents the following equation in the continuous time:
 * \f[
 * \dot{x} = - k x
 * \f]
 * In the discrete time the following dynamics assigns the current state to the next state:
 * \f[
 * x_{k+1} = x_{k} + \Delta t (- k x_{k})
 * \f]
 */
class ExternalContactStateDynamics : public Dynamics
{
    Eigen::VectorXd m_currentState; /**< Current state. */
    std::string m_name; /**< Name of dynamics. */
    std::vector<std::string> m_elements = {}; /**< Elements composing the variable vector. */
    System::VariablesHandler m_stateVariableHandler; /**< Variable handler describing the variables
                                                        and the sizes in the ukf state vector. */
    std::chrono::nanoseconds m_dT{std::chrono::nanoseconds::zero()}; /**< Sampling time. */
    Eigen::VectorXd m_k; /**< Model constant. */

    /**
     * Controls whether the variable handler contains the variables on which the dynamics depend.
     * @return True in case all the dependencies are contained in the variable handler, false
     * otherwise.
     */
    bool checkStateVariableHandler() override;

public:
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
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler, const std::string& name) override;

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
    bool
    setSubModels(const std::vector<SubModel>& subModelList,
                 const std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList) override;

    /**
     * Update the content of the element.
     * @return True in case of success, false otherwise.
     */
    bool update() override;

    /**
     * Set the state of the ukf needed to update the dynamics of the state/measurement variable
     * associated to ths object.
     * @param ukfState reference to the ukf state.
     */
    void setState(const Eigen::Ref<const Eigen::VectorXd> ukfState) override;

    /**
     * Set a `UKFInput` object.
     * @param ukfInput reference to the UKFInput struct.
     */
    void setInput(const UKFInput& ukfInput) override;

}; // class ExternalContactStateDynamics

BLF_REGISTER_UKF_DYNAMICS(ExternalContactStateDynamics,
                          ::BipedalLocomotion::Estimators::RobotDynamicsEstimator::Dynamics);

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_EXTERNAL_CONTACT_STATE_DYNAMICS_H
