/**
 * @file FrictionTorqueStateDynamics.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <Eigen/Dense>

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/FrictionTorqueStateDynamics.h>

namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

RDE::FrictionTorqueStateDynamics::FrictionTorqueStateDynamics() = default;

RDE::FrictionTorqueStateDynamics::~FrictionTorqueStateDynamics() = default;

bool RDE::FrictionTorqueStateDynamics::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[FrictionTorqueStateDynamics::initialize]";

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", errorPrefix);
        return false;
    }

    // Set the state dynamics name
    if (!ptr->getParameter("name", m_name))
    {
        log()->error("{} Error while retrieving the name variable.", errorPrefix);
        return false;
    }

    // Set the state process covariance
    if (!ptr->getParameter("covariance", m_covariances))
    {
        log()->error("{} Error while retrieving the covariance variable.", errorPrefix);
        return false;
    }

    // Set the state initial covariance
    if (!ptr->getParameter("initial_covariance", m_initialCovariances))
    {
        log()->error("{} Error while retrieving the initial_covariance variable.", errorPrefix);
        return false;
    }

    // Set the dynamic model type
    if (!ptr->getParameter("dynamic_model", m_dynamicModel))
    {
        log()->error("{} Error while retrieving the dynamic_model variable.", errorPrefix);
        return false;
    }

    // Set the list of elements if it exists
    if (!ptr->getParameter("elements", m_elements))
    {
        log()->info("{} Variable elements not found.", errorPrefix);
        m_elements = {};
    }

    // Set the friction parameters
    if (!ptr->getParameter("friction_k0", m_k0))
    {
        log()->error("{} Error while retrieving the friction_k0 variable.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("friction_k1", m_k1))
    {
        log()->error("{} Error while retrieving the friction_k1 variable.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("friction_k2", m_k2))
    {
        log()->error("{} Error while retrieving the friction_k2 variable.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("sampling_time", m_dT))
    {
        log()->error("{} Error while retrieving the sampling_time variable.", errorPrefix);
        return false;
    }

    m_description = "Friction torque state dynamics";

    m_isInitialized = true;

    return true;
}

bool RDE::FrictionTorqueStateDynamics::finalize(const System::VariablesHandler &stateVariableHandler)
{
    constexpr auto errorPrefix = "[FrictionTorqueStateDynamics::finalize]";

    if (!m_isInitialized)
    {
        log()->error("{} Please initialize the dynamics before calling finalize.", errorPrefix);
        return false;
    }

    if (stateVariableHandler.getNumberOfVariables() == 0)
    {
        log()->error("{} The state variable handler is empty.", errorPrefix);
        return false;
    }

    m_stateVariableHandler = stateVariableHandler;

    if (!checkStateVariableHandler())
    {
        log()->error("{} The state variable handler is not valid.", errorPrefix);
        return false;
    }

    m_size = m_covariances.size();

    m_motorTorqueFullModel.resize(m_stateVariableHandler.getVariable("tau_m").size);
    m_motorTorqueFullModel.setZero();

    m_frictionTorqueFullModel.resize(m_stateVariableHandler.getVariable("tau_F").size);
    m_frictionTorqueFullModel.setZero();

    m_jointAccelerationFullModel.resize(m_stateVariableHandler.getVariable("ds").size);
    m_jointAccelerationFullModel.setZero();

    m_jointVelocityFullModel.resize(m_stateVariableHandler.getVariable("ds").size);
    m_jointVelocityFullModel.setZero();

    m_subModelUpdatedJointAcceleration.resize(m_nrOfSubDynamics);

    for (int idx = 0; idx < m_nrOfSubDynamics; idx++)
    {
        m_subModelUpdatedJointAcceleration[idx].resize(m_subDynamics[idx]->subModel.getJointMapping().size());
        m_subModelUpdatedJointAcceleration[idx].setZero();
    }

    m_currentFrictionTorque.resize(m_size);
    m_currentFrictionTorque.setZero();

    m_updatedVariable.resize(m_size);
    m_updatedVariable.setZero();

    m_tanhArgument.resize(m_size);
    m_tanhArgument.setZero();

    m_tanh.resize(m_size);
    m_tanh.setZero();

    m_k0k1.resize(m_size);
    m_k0k1.setZero();

    m_argParenthesis.resize(m_size);
    m_argParenthesis.setZero();

    m_dotTauF.resize(m_size);
    m_dotTauF.setZero();

    return true;
}

bool RDE::FrictionTorqueStateDynamics::setSubModels(const std::vector<SubModel>& subModelList, const std::vector<std::shared_ptr<SubModelKinDynWrapper>>& kinDynWrapperList)
{
    constexpr auto errorPrefix = "[FrictionTorqueStateDynamics::setSubModels]";

    for (int subModelIdx = 0; subModelIdx < subModelList.size(); subModelIdx++)
    {
        if (subModelList[subModelIdx].getModel().getNrOfDOFs() > 0)
        {
            m_subDynamics.emplace_back(std::make_unique<RDE::SubModelDynamics>());

            if(!m_subDynamics[subModelIdx]->setSubModel(subModelList[subModelIdx]))
            {
                log()->error("{} The submodel at index {} is not valid.", errorPrefix, subModelIdx);
                return false;
            }

            if(!m_subDynamics[subModelIdx]->setKinDynWrapper(kinDynWrapperList[subModelIdx]))
            {
                log()->error("{} The submodel kindyn wrapper at index {} is not valid.", errorPrefix, subModelIdx);
                return false;
            }
        }
    }

    m_nrOfSubDynamics = m_subDynamics.size();

    m_isSubModelListSet = true;

    for (int subModelIdx = 0; subModelIdx < m_nrOfSubDynamics; subModelIdx++)
    {
        if (!m_subDynamics.at(subModelIdx)->initialize())
        {
            log()->error("{} Cannot initialize the joint velocity dynamics of the sub-model {}.", errorPrefix, subModelIdx);
            return false;
        }
    }

    return true;
}

bool RDE::FrictionTorqueStateDynamics::checkStateVariableHandler()
{
    constexpr auto errorPrefix = "[FrictionTorqueStateDynamics::checkStateVariableHandler]";

    if (!m_isSubModelListSet)
    {
        log()->error("{} Set the sub-model list before setting the variable handler", errorPrefix);
        return false;
    }

    if (!m_stateVariableHandler.getVariable("tau_m").isValid())
    {
        log()->error("{} The variable handler does not contain the expected state with name `tau_m`.", errorPrefix);
        return false;
    }

    // Check if the variable handler contains the variables used by this dynamics
    if (!m_stateVariableHandler.getVariable("tau_F").isValid())
    {
        log()->error("{} The variable handler does not contain the expected state with name `tau_F`.", errorPrefix);
        return false;
    }

    if (!m_stateVariableHandler.getVariable("ds").isValid())
    {
        log()->error("{} The variable handler does not contain the expected state with name `ds`.", errorPrefix);
        return false;
    }

    // check if all the sensors are part of the sub-model
    for (int subModelIdx = 0; subModelIdx < m_nrOfSubDynamics; subModelIdx++)
    {
        for (int ftIdx = 0; ftIdx < m_subDynamics.at(subModelIdx)->subModel.getNrOfFTSensor(); ftIdx++)
        {
            if (!m_stateVariableHandler.getVariable(m_subDynamics.at(subModelIdx)->subModel.getFTSensor(ftIdx).name).isValid())
            {
                log()->error("{} The variable handler does not contain the expected state with name `{}`.", errorPrefix, m_subDynamics.at(subModelIdx)->subModel.getFTSensor(ftIdx).name);
                return false;
            }
        }

        for (int contactIdx = 0; contactIdx < m_subDynamics.at(subModelIdx)->subModel.getNrOfExternalContact(); contactIdx++)
        {
            if (!m_stateVariableHandler.getVariable(m_subDynamics.at(subModelIdx)->subModel.getExternalContact(contactIdx)).isValid())
            {
                log()->error("{} The variable handler does not contain the expected state with name `{}`.", errorPrefix, m_subDynamics.at(subModelIdx)->subModel.getExternalContact(contactIdx));
                return false;
            }
        }
    }

    return true;
}

// TODO
// Change the model
bool RDE::FrictionTorqueStateDynamics::update()
{
    constexpr auto errorPrefix = "[FrictionTorqueStateDynamics::update]";

    // compute joint acceleration per each sub-model
    for (int subDynamicsIdx = 0; subDynamicsIdx < m_subDynamics.size(); subDynamicsIdx++)
    {
        if (!m_subDynamics[subDynamicsIdx]->update(m_ukfInput.robotBaseAcceleration, m_jointAccelerationFullModel, m_subModelUpdatedJointAcceleration[subDynamicsIdx]))
        {
            log()->error("{} Error updating the joint velocity dynamics for the sub-model {}.", errorPrefix, subDynamicsIdx);
            return false;
        }

        for (int jointIdx = 0; jointIdx < m_subDynamics[subDynamicsIdx]->subModel.getJointMapping().size(); jointIdx++)
        {
            m_jointAccelerationFullModel[m_subDynamics[subDynamicsIdx]->subModel.getJointMapping()[jointIdx]] = m_subModelUpdatedJointAcceleration[subDynamicsIdx][jointIdx];
        }
    }

    // k_{1} \dot{s,k}
    m_tanhArgument = m_k1.array() * m_jointVelocityFullModel.array();

    // tanh (k_{1} \dot{s,k}))
    m_tanh = m_tanhArgument.array().tanh();

    // 1 - tanh^{2} (k_{1} \dot{s,k})
    m_argParenthesis = 1 - m_tanh.array().square();

    //  k_{0} k_{1}
    m_k0k1 = m_k0.array() * m_k1.array();

    // \ddot{s,k} ( k_{2} + k_{0} k_{1} (1 - tanh^{2} (k_{1} \dot{s,k})) )
    m_dotTauF = m_jointAccelerationFullModel.array() * ( m_k2.array() + m_k0k1.array() * m_argParenthesis.array() );

    // \tau_{F,k+1} = \tau_{F,k} + \Delta T * \dot{\tau_{F,k}}
    m_updatedVariable = m_currentFrictionTorque + m_dT * m_dotTauF;

    return true;
}

void RDE::FrictionTorqueStateDynamics::setState(const Eigen::Ref<const Eigen::VectorXd> ukfState)
{
    m_jointVelocityFullModel = ukfState.segment(m_stateVariableHandler.getVariable("ds").offset,
                                                m_stateVariableHandler.getVariable("ds").size);

    m_motorTorqueFullModel = ukfState.segment(m_stateVariableHandler.getVariable("tau_m").offset,
                                              m_stateVariableHandler.getVariable("tau_m").size);

    m_frictionTorqueFullModel = ukfState.segment(m_stateVariableHandler.getVariable("tau_F").offset,
                                                 m_stateVariableHandler.getVariable("tau_F").size);

    for (int subDynamicsIdx = 0; subDynamicsIdx < m_subDynamics.size(); subDynamicsIdx++)
    {
        m_subDynamics.at(subDynamicsIdx)->setState(ukfState,
                                                   m_jointVelocityFullModel,
                                                   m_motorTorqueFullModel,
                                                   m_frictionTorqueFullModel,
                                                   m_stateVariableHandler);
    }
}

void RDE::FrictionTorqueStateDynamics::setInput(const UKFInput& ukfInput)
{
    m_ukfInput = ukfInput;
}
