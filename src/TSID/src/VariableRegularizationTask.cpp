/**
 * @file VariableRegularizationTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/TSID/VariableRegularizationTask.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/Core/EigenHelpers.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::TSID;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion;

bool VariableRegularizationTask::setVariablesHandler(const VariablesHandler& variablesHandler)
{
    constexpr auto errorPrefix = "[VariableRegularizationTask::setVariablesHandler]";

    System::VariablesHandler::VariableDescription variable;

    if (!m_isInitialized)
    {
        log()->error("{} The task is not initialized. Please call initialize method.", errorPrefix);
        return false;
    }

    if (!variablesHandler.getVariable(m_variableName, variable))
    {
        log()->error("{} Error while retrieving the variable named {}.",
                     errorPrefix,
                     m_variableName);
        return false;
    }

    // resize the matrices
    m_A.resize(m_controlledElements.size(), variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_controlledElements.size());

    // A is constant
    for (int i = 0; i < m_controlledElements.size(); i++)
    {
        const auto& element = m_controlledElements[i];
        auto index = variable.getElementIndex(m_controlledElements[i]);
        if (index < 0)
        {
            log()->error("{} Unable to find the element named {} in the variable {}",
                         m_variableName,
                         element,
                         errorPrefix);
            return false;
        }

        m_A(i, index) = 1;
    }

    return true;
}

bool VariableRegularizationTask::initialize(std::weak_ptr<const IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[VariableRegularizationTask::initialize]";

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("variable_name", m_variableName))
    {
        log()->error("{} Error while retrieving the variable name.", errorPrefix);
        return false;
    }

    // set the description
    m_description = "Variable Regularization Task [variable: " + m_variableName;

    if (!ptr->getParameter("elements_name", m_controlledElements))
    {
        log()->error("{} Error while retrieving the elements variable names.", errorPrefix);
        return false;
    }
    for (const auto& element : m_controlledElements)
    {
        m_description += " " + element;
    }
    m_description += "]";

    m_isInitialized = true;
    return true;
}

bool VariableRegularizationTask::setSetPoint(Eigen::Ref<const Eigen::VectorXd> setPoint)
{
    if (setPoint.size() != m_controlledElements.size())
    {
        m_isValid = false;

        log()->error("[VariableRegularizationTask::setSetPoint] the size of the set point vector "
                     "is different from the size of the controlled elements. Expected: {}. Passed: "
                     "{}",
                     m_controlledElements.size(),
                     setPoint.size());
        return false;
    }
    m_isValid = true;
    m_b = setPoint;
    return true;
}

std::size_t VariableRegularizationTask::size() const
{
    constexpr auto errorMessage = "[VariableRegularizationTask::size] Please call initialize "
                                  "method before. "
                                  "A size equal to zero will be returned.";

    if (!m_isInitialized)
    {
        log()->warn(errorMessage);
        return 0;
    }
    return m_controlledElements.size();
}

VariableRegularizationTask::Type VariableRegularizationTask::type() const
{
    return Type::equality;
}

bool VariableRegularizationTask::isValid() const
{
    return m_isValid;
}
