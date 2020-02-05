/**
 * @file OptimizationProblemElements.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/OptimizationProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/Weight.h>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

// constraint

Constraints::EqualityConstraint::EqualityConstraint(ControlTask* const element,
                                                    const iDynTree::IndexRange& indexRange)
    : element(element)
    , indexRange(indexRange)
{
}

Constraints::InequalityConstraint::InequalityConstraint(InequalityConstraintElement* const element,
                                                        const iDynTree::IndexRange& indexRange)
    : element(element)
    , indexRange(indexRange)
{
}

iDynTree::VectorDynSize& Constraints::Bounds::lowerBound()
{
    return this->first;
}

const iDynTree::VectorDynSize& Constraints::Bounds::lowerBound() const
{
    return this->first;
}

iDynTree::VectorDynSize& Constraints::Bounds::upperBound()
{
    return this->second;
}

const iDynTree::VectorDynSize& Constraints::Bounds::upperBound() const
{
    return this->second;
}

Constraints::Constraints(const VariableHandler& handler)
{
    m_numberOfVariables = handler.getNumberOfVariables();
}

int Constraints::getNextConstraintIndex() const
{
    int index = 0;
    if (!m_equalityConstrains.empty() && m_inequalityConstrains.empty())
        index = m_equalityConstrains.back().indexRange.offset
                + m_equalityConstrains.back().indexRange.size;
    else if (m_equalityConstrains.empty() && !m_inequalityConstrains.empty())
        index = m_inequalityConstrains.back().indexRange.offset
                + m_inequalityConstrains.back().indexRange.size;
    else if (!m_equalityConstrains.empty() && !m_inequalityConstrains.empty())
        index = std::max(m_inequalityConstrains.back().indexRange.offset
                             + m_inequalityConstrains.back().indexRange.size,
                         m_equalityConstrains.back().indexRange.offset
                             + m_equalityConstrains.back().indexRange.size);

    return index;
}

void Constraints::addConstraint(ControlTask* element)
{
    iDynTree::IndexRange temp;
    temp.offset = m_numberOfConstraints;
    temp.size = element->getSize();

    // add the equality constraint
    m_equalityConstrains.emplace_back(element, temp);

    // increase the number of constraint
    m_numberOfConstraints += temp.size;

    // resize the matrices
    m_lowerBound.resize(m_numberOfConstraints);
    m_upperBound.resize(m_numberOfConstraints);
    m_constraintMatrix.resize(m_numberOfConstraints, m_numberOfVariables);
}

void Constraints::addConstraint(InequalityConstraintElement* element)
{
    iDynTree::IndexRange temp;
    temp.offset = m_numberOfConstraints;
    temp.size = element->getSize();

    // add the inequality constraint
    m_inequalityConstrains.emplace_back(element, temp);

    // increase the number of constraint
    m_numberOfConstraints += temp.size;

    // resize the matrices
    m_lowerBound.resize(m_numberOfConstraints);
    m_upperBound.resize(m_numberOfConstraints);
    m_constraintMatrix.resize(m_numberOfConstraints, m_numberOfVariables);
}

Constraints::Bounds Constraints::getBounds()
{
    // equality constraint
    for (EqualityConstraint& constraint : m_equalityConstrains)
    {
        const iDynTree::VectorDynSize& b = constraint.element->getB();

        iDynTree::toEigen(m_lowerBound)
            .segment(constraint.indexRange.offset, constraint.indexRange.size)
            = iDynTree::toEigen(b);
        iDynTree::toEigen(m_upperBound)
            .segment(constraint.indexRange.offset, constraint.indexRange.size)
            = iDynTree::toEigen(b);
    }

    // inequality constraint
    for (InequalityConstraint& constraint : m_inequalityConstrains)
    {
        const iDynTree::VectorDynSize& upperBound = constraint.element->getUpperBound();
        const iDynTree::VectorDynSize& lowerBound = constraint.element->getLowerBound();

        iDynTree::toEigen(m_lowerBound)
            .segment(constraint.indexRange.offset, constraint.indexRange.size)
            = iDynTree::toEigen(lowerBound);
        iDynTree::toEigen(m_upperBound)
            .segment(constraint.indexRange.offset, constraint.indexRange.size)
            = iDynTree::toEigen(upperBound);
    }

    return Constraints::Bounds(m_lowerBound, m_upperBound);
}

const iDynTree::MatrixDynSize& Constraints::getConstraintMatrix()
{
    // equality constraint
    for (EqualityConstraint& constraint : m_equalityConstrains)
        iDynTree::toEigen(m_constraintMatrix)
            .middleRows(constraint.indexRange.offset, constraint.indexRange.size)
            = iDynTree::toEigen(constraint.element->getA());

    // inequality constraint
    for (InequalityConstraint& constraint : m_inequalityConstrains)
        iDynTree::toEigen(m_constraintMatrix)
            .middleRows(constraint.indexRange.offset, constraint.indexRange.size)
            = iDynTree::toEigen(constraint.element->getA());

    return m_constraintMatrix;
}

int Constraints::getNumberOfConstraints() const
{
    return m_numberOfConstraints;
}

const std::vector<Constraints::EqualityConstraint>& Constraints::getEqualityConstraints() const
{
    return m_equalityConstrains;
}
const std::vector<Constraints::InequalityConstraint>& Constraints::getInequalityConstraints() const
{
    return m_inequalityConstrains;
}

// CostFunction

iDynTree::MatrixDynSize& CostFunction::Elements::hessian()
{
    return this->first;
}

const iDynTree::MatrixDynSize& CostFunction::Elements::hessian() const
{
    return this->first;
}

iDynTree::VectorDynSize& CostFunction::Elements::gradient()
{
    return this->second;
}

const iDynTree::VectorDynSize& CostFunction::Elements::gradient() const
{
    return this->second;
}

CostFunction::CostFunctionElement::CostFunctionElement(ControlTask* const element,
                                                       const Weight<iDynTree::VectorDynSize>& weight)
    : element(element)
    , weight(weight)
{
}

CostFunction::CostFunction(const VariableHandler& handler)
{
    int numberOfVariables = handler.getNumberOfVariables();

    m_gradient.resize(numberOfVariables);
    m_hessianMatrix.resize(numberOfVariables, numberOfVariables);

    m_hessianMatrix.zero();
    m_gradient.zero();
}

void CostFunction::addCostFunction(ControlTask* const element,
                                   const Weight<iDynTree::VectorDynSize>& weight,
                                   const std::string& name)
{
    m_costFunctionElements.insert(
        {name, CostFunctionElement(element, weight)});
}

CostFunction::Elements CostFunction::getElements()
{

    if (m_costFunctionElements.size() > 0)
    {
        auto iter = m_costFunctionElements.begin();
        const iDynTree::MatrixDynSize& A = iter->second.element->getA();
        const iDynTree::VectorDynSize& b = iter->second.element->getB();

        iDynTree::toEigen(m_hessianMatrix) = iDynTree::toEigen(A).transpose()
                                             * iDynTree::toEigen(iter->second.weight.getWeight()).asDiagonal()
                                             * iDynTree::toEigen(A);

        iDynTree::toEigen(m_gradient) = -iDynTree::toEigen(A).transpose()
                                        * iDynTree::toEigen(iter->second.weight.getWeight()).asDiagonal()
                                        * iDynTree::toEigen(b);

        for (std::advance(iter, 1); iter != m_costFunctionElements.end(); ++iter)
        {
            const iDynTree::MatrixDynSize& A = iter->second.element->getA();
            const iDynTree::VectorDynSize& b = iter->second.element->getB();

            iDynTree::toEigen(m_hessianMatrix)
                += iDynTree::toEigen(A).transpose()
                   * iDynTree::toEigen(iter->second.weight.getWeight()).asDiagonal() * iDynTree::toEigen(A);

            iDynTree::toEigen(m_gradient) -= iDynTree::toEigen(A).transpose()
                                             * iDynTree::toEigen(iter->second.weight.getWeight()).asDiagonal()
                                             * iDynTree::toEigen(b);
        }
    }

    return CostFunction::Elements(m_hessianMatrix, m_gradient);
}

bool CostFunction::setWeight(const Weight<iDynTree::VectorDynSize>& weight, const std::string& elementName)
{
    auto element = m_costFunctionElements.find(elementName);
    if (element == m_costFunctionElements.end())
    {
        std::cerr << "[CostFunction::setWeight] Unable to find the CostFunction element named "
                  << elementName << std::endl;
        return false;
    }

    element->second.weight = weight;

    return true;
}

const std::unordered_map<std::string, CostFunction::CostFunctionElement>&
CostFunction::getCostFunctions() const
{
    return m_costFunctionElements;
}
