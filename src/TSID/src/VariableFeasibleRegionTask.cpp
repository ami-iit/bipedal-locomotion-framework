/**
 * @file VariableFeasibleRegionTask.cpp
 * @authors Roberto Mauceri
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TSID/VariableFeasibleRegionTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::TSID;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion;

bool VariableFeasibleRegionTask::setVariablesHandler(const VariablesHandler& variablesHandler)
{
    constexpr auto errorPrefix = "[VariableFeasibleRegionTask::setVariablesHandler]";

    System::VariablesHandler::VariableDescription variable;

    if (!m_isInitialized)
    {
        log()->error("{} The task is not initialized. Please call initialize() method.",
                     errorPrefix);
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
        log()->error("{} The expected size is greater than the one stored in the variable named "
                     "{}.",
                     errorPrefix,
                     m_variableName);
        return false;
    }
    m_NumberOfVariables = variablesHandler.getNumberOfVariables();

    // m_S (selection matrix)
    m_S.resize(m_variableSize, m_NumberOfVariables);
    m_S.setZero();
    // m_S is constant
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
        // iDynTree::toEigen(this->subA(variable)).setIdentity(); // devo sostituirlo col comando
        // sotto?
        m_S = Eigen::MatrixXd::Identity(m_variableSize, m_variableSize);
    }
    return true;
}

bool VariableFeasibleRegionTask::initialize(std::weak_ptr<const IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[VariableFeasibleRegionTask::initialize]";

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
    // set the variable size
    m_variableSize = variableSize;

    // set the description
    m_description = "Variable Feasible Region Task [variable: " + m_variableName + ", elements: ";

    if (!ptr->getParameter("elements_name", m_controlledElements))
    {
        m_description += "All";
        log()->debug("{} The elements_name is not set. All the {} elements will be considered.",
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

bool VariableFeasibleRegionTask::setFeasibleRegion(Eigen::Ref<const Eigen::MatrixXd> C,
                                                   Eigen::Ref<const Eigen::VectorXd> l,
                                                   Eigen::Ref<const Eigen::VectorXd> u)
{
    m_isValid = false;
    constexpr auto errorPrefix = "[VariableFeasibleRegionTask::setFeasibleRegion]";
    // check if the size of the matrices is correct
    if (C.cols() != m_variableSize)
    {
        log()->error("{} The number of columns of the matrix C is not correct. Expected: {}, "
                     "Passed: {}.",
                     errorPrefix,
                     m_variableSize,
                     C.cols());
        return false;
    }
    if (C.rows() != l.size())
    {
        log()->error("{} The size of the vector l is not correct. Expected: {}, Passed: {}.",
                     errorPrefix,
                     C.rows(),
                     l.size());
        return false;
    }
    if (C.rows() != u.size())
    {
        log()->error("{} The size of the vector u is not correct. Expected: {}, Passed: {}.",
                     errorPrefix,
                     C.rows(),
                     u.size());
        return false;
    }
    if (!(u.array() >= l.array()).all())
    {
        log()->error("{} The elements of the vector u must be greater than or equal to"
                     " the elements of the vector l.",
                     errorPrefix);
        return false;
    }
    m_isValid = true;

    // resize the matrices
    m_T.resize(2 * C.rows(), C.cols());
    m_A.resize(2 * C.rows(), m_NumberOfVariables);
    m_b.resize(2 * C.rows());

    // set the matrices
    m_T << C, -C;
    m_A = m_T * m_S;
    m_b << u, -l;
    return true;
}

std::size_t VariableFeasibleRegionTask::size() const
{
    return m_b.size();
}

VariableFeasibleRegionTask::Type VariableFeasibleRegionTask::type() const
{
    return Type::inequality;
}

bool VariableFeasibleRegionTask::isValid() const
{
    return m_isValid;
}
