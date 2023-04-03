/**
 * @file ZeroVelocityDynamics.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/ZeroVelocityDynamics.h>

namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

bool RDE::ZeroVelocityDynamics::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[ZeroVelocityDynamics::initialize]";

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
        log()->debug("{} Variable initial_covariance not found.", errorPrefix);
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
        log()->debug("{} Variable elements not found.", errorPrefix);
        m_elements = {};
    }

    // Set the bias related variables if use_bias is true
    if (!ptr->getParameter("use_bias", m_useBias))
    {
        log()->debug("{} Variable use_bias not found. Set to false by default.", errorPrefix);
    }
    else
    {
        m_useBias = true;
        m_biasVariableName = m_name + "_bias";
    }

    m_description = "Zero velocity dynamics";

    m_isInitialized = true;

    return true;
}

bool RDE::ZeroVelocityDynamics::finalize(const System::VariablesHandler &stateVariableHandler)
{
    constexpr auto errorPrefix = "[ZeroVelocityDynamics::finalize]";

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

    m_currentState.resize(m_size);
    m_currentState.setZero();

    m_updatedVariable.resize(m_size);
    m_updatedVariable.setZero();

    // Set the bias related variables if use_bias is true
    if (m_useBias)
    {
        m_bias.resize(m_size);
        m_bias.setZero();
    }

    return true;
}

bool RDE::ZeroVelocityDynamics::setSubModels(const std::vector<RDE::SubModel>& subModelList, const std::vector<std::shared_ptr<RDE::SubModelKinDynWrapper>>& kinDynWrapperList)
{
    return true;
}

bool RDE::ZeroVelocityDynamics::checkStateVariableHandler()
{
    constexpr auto errorPrefix = "[FrictionTorqueStateDynamics::checkStateVariableHandler]";

    // Check if the variable handler contains the variables used by this dynamics
    if (!m_stateVariableHandler.getVariable(m_name).isValid())
    {
        log()->error("{} The variable handler does not contain the expected state with name `{}`.", errorPrefix, m_name);
        return false;
    }

    if (m_useBias)
    {
        if (!m_stateVariableHandler.getVariable(m_biasVariableName).isValid())
        {
            log()->error("{} The variable handler does not contain the expected state name {}.", errorPrefix, m_biasVariableName);
            return false;
        }
    }

    return true;
}

bool RDE::ZeroVelocityDynamics::update()
{
    if (m_useBias)
    {
        m_updatedVariable = m_currentState + m_bias;
    }
    else
    {
        m_updatedVariable = m_currentState;
    }

    return true;
}

void RDE::ZeroVelocityDynamics::setState(const Eigen::Ref<const Eigen::VectorXd> ukfState)
{
    m_currentState = ukfState.segment(m_stateVariableHandler.getVariable(m_name).offset,
                                      m_stateVariableHandler.getVariable(m_name).size);

    if (m_useBias)
    {
        m_bias = ukfState.segment(m_stateVariableHandler.getVariable(m_biasVariableName).offset,
                                  m_stateVariableHandler.getVariable(m_biasVariableName).size);
    }
}

void RDE::ZeroVelocityDynamics::setInput(const UKFInput& ukfInput)
{
    return;
}
