/**
 * @file VariableLinearTask.cpp
 * @authors Roberto Mauceri
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TSID/VariableLinearTask.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::TSID;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion;

bool VariableLinearTask::setVariablesHandler(const VariablesHandler& variablesHandler)
{
    constexpr auto errorPrefix = "[VariableLinearTask::setVariablesHandler]";

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
    m_S.resize(2 * m_variableSize, variablesHandler.getNumberOfVariables());
    m_S.setZero();
    m_b.resize(2 * m_variableSize);

    // S (selection matrix) is constant
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

            m_S(i, index) = 1;
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

bool VariableLinearTask::initialize(std::weak_ptr<const IParametersHandler> paramHandler)
{

}

bool VariableLinearTask::setUpperBound(Eigen::Ref<const Eigen::VectorXd> setPoint)
{

}

std::size_t VariableLinearTask::size() const
{
    return m_b.size();
}

VariableLinearTask::Type VariableLinearTask::type() const
{
    return Type::inequality; // By default, the "<" operator is considered
}

bool VariableLinearTask::isValid() const
{
    return m_isValid;
}
