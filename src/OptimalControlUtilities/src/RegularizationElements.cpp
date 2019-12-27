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

    iDynTree::IndexRange variableIndex = handler.getVariable(variableName);

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
    const VariableHandler& handler,
    const std::string& variableName)
    : RegularizationElement(kinDyn, handler, variableName)
{
    m_name = "Regularization with control element (variable: " + variableName + ")";

    iDynTree::IndexRange variableIndex = handler.getVariable(variableName);

    // instantiate controller
    m_pd = std::make_unique<LinearPD<iDynTree::VectorDynSize>>();

    if (!variableIndex.isValid())
        throw std::runtime_error("[RegularizationWithControlElement::"
                                 "RegularizationWithControlElement] Undefined variable named "
                                 + variableName + " in the variableHandler");
}

void RegularizationWithControlElement::setDesiredTrajectory(const iDynTree::VectorDynSize& acceleration,
                                                            const iDynTree::VectorDynSize& velocity,
                                                            const iDynTree::VectorDynSize& position)
{
    m_pd->setDesiredTrajectory(acceleration, velocity, position);
}

void RegularizationWithControlElement::setState(const iDynTree::VectorDynSize& velocity,
                                                const iDynTree::VectorDynSize& position)
{
    m_pd->setFeedback(velocity, position);
}

void RegularizationWithControlElement::setPDGains(const iDynTree::VectorDynSize& kp,
                                                   const iDynTree::VectorDynSize& kd)
{
    m_pd->setGains(kp, kd);
}

const iDynTree::VectorDynSize& RegularizationWithControlElement::getB()
{
    return m_pd->getControllerOutput();
}
