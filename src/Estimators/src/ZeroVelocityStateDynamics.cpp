/**
 * @file ZeroVelocityStateDynamics.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/RobotDynamicsEstimator/ZeroVelocityStateDynamics.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

bool RDE::ZeroVelocityStateDynamics::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler,
    const std::string& name)
{
    constexpr auto errorPrefix = "[ZeroVelocityStateDynamics::initialize]";

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", errorPrefix);
        return false;
    }

    m_name = name;

    // Set the state process covariance
    if (!ptr->getParameter("covariance", m_covariances))
    {
        log()->error("{} Error while retrieving the covariance variable.", errorPrefix);
        return false;
    }

    // Set the state initial covariance
    if (!ptr->getParameter("initial_covariance", m_initialCovariances))
    {
        log()->error("{} Variable initial_covariance not found.", errorPrefix);
        return false;
    }

    // Set the list of elements if it exists
    if (!ptr->getParameter("elements", m_elements))
    {
        log()->debug("{} Variable elements not found.", errorPrefix);
    }

    m_description = "Zero velocity dynamics";

    m_isInitialized = true;

    return true;
}

bool RDE::ZeroVelocityStateDynamics::finalize(const System::VariablesHandler& stateVariableHandler)
{
    constexpr auto errorPrefix = "[ZeroVelocityStateDynamics::finalize]";

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

    return true;
}

bool RDE::ZeroVelocityStateDynamics::setSubModels(
    const std::vector<RDE::SubModel>& /*subModelList*/,
    const std::vector<std::shared_ptr<RDE::KinDynWrapper>>& /*kinDynWrapperList*/)
{
    return true;
}

bool RDE::ZeroVelocityStateDynamics::checkStateVariableHandler()
{
    constexpr auto errorPrefix = "[ZeroVelocityStateDynamics::checkStateVariableHandler]";

    // Check if the variable handler contains the variables used by this dynamics
    if (!m_stateVariableHandler.getVariable(m_name).isValid())
    {
        log()->error("{} The variable handler does not contain the expected state with name `{}`.",
                     errorPrefix,
                     m_name);
        return false;
    }

    return true;
}

bool RDE::ZeroVelocityStateDynamics::update()
{
    m_updatedVariable = m_currentState;

    return true;
}

void RDE::ZeroVelocityStateDynamics::setState(const Eigen::Ref<const Eigen::VectorXd> ukfState)
{
    m_currentState = ukfState.segment(m_stateVariableHandler.getVariable(m_name).offset,
                                      m_stateVariableHandler.getVariable(m_name).size);
}

void RDE::ZeroVelocityStateDynamics::setInput(const UKFInput& /*ukfInput*/)
{
    return;
}
