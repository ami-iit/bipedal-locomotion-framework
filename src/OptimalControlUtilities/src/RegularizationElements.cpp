/**
 * @file RegularizationElements.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <stdexcept>

#include <iDynTree/Core/EigenHelpers.h>


#include <BipedalLocomotionControllers/OptimalControlUtilities/RegularizationElements.h>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

// RegularizationElement
RegularizationElement::RegularizationElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                             const VariableHandler& handler,
                                             const std::string& variableName)
    : ControlTask(kinDyn)
{
    m_name = "Regularization element (variable: " + variableName + ")";

    const iDynTree::IndexRange& variableIndex = handler.getVariable(variableName);

    if (!variableIndex.isValid())
        throw std::runtime_error("[RegularizationElement::RegularizationElement] Undefined "
                                 "variable named "
                                 + variableName + " in the variableHandler");

    // resize and set the matrices
    m_A.resize(variableIndex.size, handler.getNumberOfVariables());
    m_A.zero();
    m_b.resize(variableIndex.size);
    m_b.zero();

    iDynTree::toEigen(m_A)
        .block(0, variableIndex.offset, variableIndex.size, variableIndex.size)
        .setIdentity();
}

// RegularizationWithControlElement
RegularizationWithControlElement::RegularizationWithControlElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const LinearPD<iDynTree::VectorDynSize>& controller,
    const VariableHandler& handler,
    const std::string& variableName)
    : RegularizationElement(kinDyn, handler, variableName)
    , m_pd(controller)
{
    m_name = "Regularization with control element (variable: " + variableName + ")";

    const iDynTree::IndexRange& variableIndex = handler.getVariable(variableName);

    if (!variableIndex.isValid())
        throw std::runtime_error("[RegularizationWithControlElement::"
                                 "RegularizationWithControlElement] Undefined variable named "
                                 + variableName + " in the variableHandler");
}

void RegularizationWithControlElement::setReference(const iDynTree::VectorDynSize& feedforward,
                                                    const iDynTree::VectorDynSize& stateDerivative,
                                                    const iDynTree::VectorDynSize& state)
{
    m_pd.setDesiredTrajectory(feedforward, stateDerivative, state);
}

void RegularizationWithControlElement::setState(const iDynTree::VectorDynSize& stateDerivative,
                                                const iDynTree::VectorDynSize& state)
{
    m_pd.setFeedback(stateDerivative, state);
}

void RegularizationWithControlElement::setPDGains(const iDynTree::VectorDynSize& kp,
                                                  const iDynTree::VectorDynSize& kd)
{
    m_pd.setGains(kp, kd);
}

const iDynTree::VectorDynSize& RegularizationWithControlElement::getB()
{
    m_b = m_pd.getControllerOutput();
    return m_b;
}
