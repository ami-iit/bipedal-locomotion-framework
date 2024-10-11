/**
 * @file VariableRegularizationTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TSID/VariableRegularizationTask.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>

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

    if (m_variableSize > variable.size)
    {
        log()->error("{} The expect size is greater than the one stored in the variable named {}. "
                     "This task is used to regularize a subset of elements of a variable.",
                     errorPrefix,
                     m_variableName);
        return false;
    }

    // resize the matrices
    m_A.resize(m_variableSize, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_variableSize);

    // A is constant
    if (m_controlledElements.size() != 0)
    {
        for (int i = 0; i < m_controlledElements.size(); i++)
        {
            const auto& element = m_controlledElements[i];
            auto index = variable.getElementIndex(m_controlledElements[i]);
            if (index < 0)
            {
                log()->error("{} Unable to find the element named {} in the variable {}",
                             errorPrefix,
                             element,
                             m_variableName);
                return false;
            }

            m_A(i, index) = 1;
        }
    } else
    {
        if (m_variableSize != variable.size)
        {
            log()->error("{} The size of the variable named {} is different from expected. "
                         "Expected: {}, Passed: {}.",
                         errorPrefix,
                         m_variableName,
                         m_variableSize,
                         variable.size);
            return false;
        }
        // if the size of the m_controlledElements vector is zero, this means that the entire
        // variable is regularized
        iDynTree::toEigen(this->subA(variable)).setIdentity();
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

    int variableSize{-1};
    if (!ptr->getParameter("variable_size", variableSize) || variableSize < 0)
    {
        log()->error("{} Error while retrieving the size of the variable.", errorPrefix);
        return false;
    }
    m_variableSize = variableSize;

    // set the description
    m_description = "Variable Regularization Task [variable: " + m_variableName + ", elements: ";

    if (!ptr->getParameter("elements_name", m_controlledElements))
    {
        m_description += "All";
        log()->debug("{} The elements_name is not set. All the {} elements will be regularized.",
                     errorPrefix,
                     m_variableSize);
    } else
    {
        if (m_variableSize != m_controlledElements.size())
        {
            log()->error("{} The size of the elements_name vector is different from the one "
                         "expected. Expected: {}, Retrieved: {}",
                         errorPrefix,
                         m_variableSize,
                         m_controlledElements.size());
            return false;
        }
        for (const auto& element : m_controlledElements)
        {
            m_description += " " + element;
        }
    }
    m_description += "]";

    m_isInitialized = true;
    return true;
}

bool VariableRegularizationTask::setSetPoint(Eigen::Ref<const Eigen::VectorXd> setPoint)
{
    m_isValid = false;
    if (setPoint.size() != m_variableSize)
    {

        log()->error("[VariableRegularizationTask::setSetPoint] the size of the set point vector "
                     "is different from the size of the controlled elements. Expected: {}. Passed: "
                     "{}",
                     m_variableSize,
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
    return m_variableSize;
}

VariableRegularizationTask::Type VariableRegularizationTask::type() const
{
    return Type::equality;
}

bool VariableRegularizationTask::isValid() const
{
    return m_isValid;
}
