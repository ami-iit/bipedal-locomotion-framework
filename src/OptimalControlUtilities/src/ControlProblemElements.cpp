/**
 * @file ControlProblemElements.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

// Control Problem Element Function
ControlProblemElement::ControlProblemElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
    : m_kinDynPtr(kinDyn)
{
}

size_t ControlProblemElement::getSize() const
{
    return m_A.rows();
}

const iDynTree::MatrixDynSize& ControlProblemElement::getA()
{
    return m_A;
}

const std::string& ControlProblemElement::getName() const
{
    return m_name;
}

// ControlTask
ControlTask::ControlTask(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
    : ControlProblemElement(kinDyn)
{
}

const iDynTree::VectorDynSize& ControlTask::getB()
{
    return m_b;
}

// InequalityConstraintElement
InequalityConstraintElement::InequalityConstraintElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
    : ControlProblemElement(kinDyn)
{
}

const iDynTree::VectorDynSize& InequalityConstraintElement::getUpperBound()
{
    return m_u;
}

const iDynTree::VectorDynSize& InequalityConstraintElement::getLowerBound()
{
    return m_l;
}
