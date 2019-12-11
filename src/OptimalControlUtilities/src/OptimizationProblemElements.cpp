/**
 * @file OptimizationProblemElements.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/OptimizationProblemElements.h>

using namespace BipedalLocomotionControllers;

// constraint

Constraints::EqualityConstraint::EqualityConstraint(
    CostFunctionOrEqualityConstraintElement* const element, const iDynTree::IndexRange& indexRange)
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

void Constraints::addConstraint(CostFunctionOrEqualityConstraintElement* element)
{
    iDynTree::IndexRange temp;
    temp.offset = m_numberOfConstraints;
    temp.size = element->getSize();

    // add the equality constraint
    m_equalityConstrains.push_back(EqualityConstraint(element, temp));

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
    m_inequalityConstrains.push_back(InequalityConstraint(element, temp));

    // increase the number of constraint
    m_numberOfConstraints += temp.size;

    // resize the matrices
    m_lowerBound.resize(m_numberOfConstraints);
    m_upperBound.resize(m_numberOfConstraints);
    m_constraintMatrix.resize(m_numberOfConstraints, m_numberOfVariables);
}

std::pair<iDynTree::VectorDynSize&, iDynTree::VectorDynSize&> Constraints::getBounds()
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

    return std::pair<iDynTree::VectorDynSize&, iDynTree::VectorDynSize&>(m_lowerBound,
                                                                         m_upperBound);
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
CostFunction::CostFunctionElement::CostFunctionElement(
    CostFunctionOrEqualityConstraintElement* const element,
    const iDynTree::VectorDynSize& weight,
    const double& weightScaling,
    const double& weightOffset)
    : element(element)
    , weight(weight)
    , weightScaling(weightScaling)
    , weightOffset(weightOffset)
{
    iDynTree::toEigen(this->weight) *= weightScaling;
    for (int i = 0; i < this->weight.size(); i++)
        this->weight(i) += weightOffset;
}

CostFunction::CostFunction(const VariableHandler& handler)
{
    int numberOfVariables = handler.getNumberOfVariables();

    m_gradient.resize(numberOfVariables);
    m_hessianMatrix.resize(numberOfVariables, numberOfVariables);

    m_hessianMatrix.zero();
    m_gradient.zero();
}

void CostFunction::addCostFunction(CostFunctionOrEqualityConstraintElement* const element,
                                   const iDynTree::VectorDynSize& weight,
                                   const double& weightScaling,
                                   const double& weightOffset,
                                   const std::string& name)
{
    m_costFunctionElements.insert(
        {name, CostFunctionElement(element, weight, weightScaling, weightOffset)});
}

std::pair<const iDynTree::MatrixDynSize&, iDynTree::VectorDynSize&> CostFunction::getElements()
{

    if (m_costFunctionElements.size() > 0)
    {
        auto iter = m_costFunctionElements.begin();
        const iDynTree::MatrixDynSize& A = iter->second.element->getA();
        const iDynTree::VectorDynSize& b = iter->second.element->getB();

        iDynTree::toEigen(m_hessianMatrix) = iDynTree::toEigen(A).transpose()
                                             * iDynTree::toEigen(iter->second.weight).asDiagonal()
                                             * iDynTree::toEigen(A);

        iDynTree::toEigen(m_gradient) = -iDynTree::toEigen(A).transpose()
                                        * iDynTree::toEigen(iter->second.weight).asDiagonal()
                                        * iDynTree::toEigen(b);

        for (std::advance(iter, 1); iter != m_costFunctionElements.end(); ++iter)
        {
            const iDynTree::MatrixDynSize& A = iter->second.element->getA();
            const iDynTree::VectorDynSize& b = iter->second.element->getB();

            iDynTree::toEigen(m_hessianMatrix)
                += iDynTree::toEigen(A).transpose()
                   * iDynTree::toEigen(iter->second.weight).asDiagonal() * iDynTree::toEigen(A);

            iDynTree::toEigen(m_gradient) -= iDynTree::toEigen(A).transpose()
                                             * iDynTree::toEigen(iter->second.weight).asDiagonal()
                                             * iDynTree::toEigen(b);
        }
    }

    return std::pair<const iDynTree::MatrixDynSize&, iDynTree::VectorDynSize&>(m_hessianMatrix,
                                                                               m_gradient);
}

bool CostFunction::setWeight(const iDynTree::VectorDynSize& weight, const std::string& elementName)
{
    auto element = m_costFunctionElements.find(elementName);
    if (element == m_costFunctionElements.end())
    {
        std::cerr << "[CostFunction::setWeight] Unable to find the CostFunction element named "
                  << elementName << std::endl;
        return false;
    }

    iDynTree::toEigen(element->second.weight)
        = element->second.weightScaling * iDynTree::toEigen(weight).cwiseAbs();
    for (int i = 0; i < element->second.weight.size(); i++)
        element->second.weight(i) += element->second.weightOffset;

    return true;
}

bool CostFunction::setWeight(const double& weight, const std::string& elementName)
{
    auto element = m_costFunctionElements.find(elementName);
    if (element == m_costFunctionElements.end())
    {
        std::cerr << "[CostFunction::setWeight] Unable to find the CostFunction element named "
                  << elementName << std::endl;
        return false;
    }

    for (int i = 0; i < element->second.weight.size(); i++)
        element->second.weight(i)
            = element->second.weightScaling * std::abs(weight) + element->second.weightOffset;

    return true;
}

const std::unordered_map<std::string, CostFunction::CostFunctionElement>&
CostFunction::getCostFunctions() const
{
    return m_costFunctionElements;
}
