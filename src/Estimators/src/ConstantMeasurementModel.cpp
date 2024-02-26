/**
 * @file ConstantMeasurementModel.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/RobotDynamicsEstimator/ConstantMeasurementModel.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

bool RDE::ConstantMeasurementModel::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler,
    const std::string& name)
{
    constexpr auto errorPrefix = "[ConstantMeasurementModel::initialize]";

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("associated_state", m_name))
    {
        log()->error("{} Error while retrieving the associated state variable.", errorPrefix);
        return false;
    }

    // Set the state process covariance
    if (!ptr->getParameter("covariance", m_covariances))
    {
        log()->error("{} Error while retrieving the covariance variable.", errorPrefix);
        return false;
    }

    // Set the list of elements if it exists
    if (!ptr->getParameter("elements", m_elements))
    {
        log()->debug("{} Variable elements not found.", errorPrefix);
    }

    // Set the bias related variables if use_bias is true
    if (!ptr->getParameter("use_bias", m_useBias))
    {
        log()->debug("{} Variable use_bias not found. Set to false by default.", errorPrefix);
    } else
    {
        m_biasVariableName = m_name + "_bias";
    }

    m_description = "Constant measurement dynamics";

    m_isInitialized = true;

    return true;
}

bool RDE::ConstantMeasurementModel::finalize(const System::VariablesHandler& stateVariableHandler)
{
    constexpr auto errorPrefix = "[ConstantMeasurementModel::finalize]";

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

bool RDE::ConstantMeasurementModel::setSubModels(
    const std::vector<RDE::SubModel>& /*subModelList*/,
    const std::vector<std::shared_ptr<RDE::KinDynWrapper>>& /*kinDynWrapperList*/)
{
    return true;
}

bool RDE::ConstantMeasurementModel::checkStateVariableHandler()
{
    constexpr auto errorPrefix = "[ConstantMeasurementModel::checkStateVariableHandler]";

    // Check if the variable handler contains the variables used by this dynamics
    if (m_useBias)
    {
        if (!m_stateVariableHandler.getVariable(m_biasVariableName).isValid())
        {
            log()->error("{} The variable handler does not contain the expected state name {}.",
                         errorPrefix,
                         m_biasVariableName);
            return false;
        }
    }

    return true;
}

bool RDE::ConstantMeasurementModel::update()
{
    if (m_useBias)
    {
        m_updatedVariable = m_currentState + m_bias;
    } else
    {
        m_updatedVariable = m_currentState;
    }

    return true;
}

void RDE::ConstantMeasurementModel::setState(const Eigen::Ref<const Eigen::VectorXd> ukfState)
{
    m_currentState = ukfState.segment(m_stateVariableHandler.getVariable(m_name).offset,
                                      m_stateVariableHandler.getVariable(m_name).size);

    if (m_useBias)
    {
        m_bias = ukfState.segment(m_stateVariableHandler.getVariable(m_biasVariableName).offset,
                                  m_stateVariableHandler.getVariable(m_biasVariableName).size);
    }
}

void RDE::ConstantMeasurementModel::setInput(const UKFInput& /*ukfInput*/)
{
    return;
}
