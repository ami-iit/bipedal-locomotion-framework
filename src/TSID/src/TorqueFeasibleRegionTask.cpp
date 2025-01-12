/**
 * @file TorqueFeasibleRegionTask.cpp
 * @authors Roberto Mauceri
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TSID/TorqueFeasibleRegionTask.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::TSID;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion;

bool TorqueFeasibleRegionTask::setVariablesHandler(const VariablesHandler& variablesHandler)
{
    constexpr auto errorPrefix = "[TorqueFeasibleRegionTask::setVariablesHandler]";

    System::VariablesHandler::VariableDescription variable;

    if (!m_isInitialized)
    {
        log()->error("{} The task is not initialized. Please call initialize() method.", errorPrefix);
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
        log()->error("{} The expected size is greater than the one stored in the variable named {}.",
                     errorPrefix,
                     m_variableName);
        return false;
    }

    // resize the matrices
    m_A.resize(2 * m_variableSize, variablesHandler.getNumberOfVariables());
    m_b.resize(2 * m_variableSize);

    // m_S (selection matrix)
    m_S.resize(m_variableSize, variablesHandler.getNumberOfVariables());
    m_S.setZero();
    // S is constant
    if (m_controlledElements.size() == 2)
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
        log()->error("{} The size of the variable named {} is different from expected. "
                        "Expected: 2, Passed: {}.",
                        errorPrefix,
                        m_variableName,
                        variable.size);
        return false;
    }

    return true;
}

bool TorqueFeasibleRegionTask::initialize(std::weak_ptr<const IParametersHandler> paramHandler)
{
   constexpr auto errorPrefix = "[TorqueFeasibleRegionTask::initialize]";

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
    m_description = "SPU Torque Limit Task [variable: " + m_variableName + ", elements: 2]";

    m_isInitialized = true;
    return true;

}

bool TorqueFeasibleRegionTask::setTorqueFeasibleRegion(
    Eigen::Ref<const Eigen::MatrixXd> Q, 
    Eigen::Ref<const Eigen::VectorXd> l, 
    Eigen::Ref<const Eigen::VectorXd> u
)
{
    constexpr auto errorPrefix = "[TorqueFeasibleRegionTask::setTorqueFeasibleRegion]";

    m_isValid = false;
    if (Q.rows() != m_variableSize || Q.rows() != m_variableSize)
    {
        log()->error("{} The matrix Q must have an order equal to {}", errorPrefix, m_variableSize);
        return false;
    }

        if (Q.rows() != m_variableSize || Q.rows() != m_variableSize)
    {
        log()->error("{} The vector l must have a dimension equal to {}", errorPrefix, m_variableSize);
        return false;
    }

        if (Q.rows() != m_variableSize || Q.rows() != m_variableSize)
    {
        log()->error("{} The vector u must have a dimension equal to {}", errorPrefix, m_variableSize);
        return false;
    }

    m_A << Q, -Q;
    m_A = m_A * m_S;

    m_b << u, -l;
    m_isValid = true;
    return true
}

std::size_t TorqueFeasibleRegionTask::size() const
{
    return m_b.size();
}

TorqueFeasibleRegionTask::Type TorqueFeasibleRegionTask::type() const
{
    return Type::inequality;    // by default, the "<" operator is considered
}

bool TorqueFeasibleRegionTask::isValid() const
{
    return m_isValid;
}
