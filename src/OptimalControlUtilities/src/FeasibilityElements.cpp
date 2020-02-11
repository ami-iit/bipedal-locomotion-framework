/**
 * @file FeasibilityElements.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <stdexcept>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/FeasibilityElements.h>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

// JointValuesFeasibilityElement
JointValuesFeasibilityElement::JointValuesFeasibilityElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                                             const VariableHandler& handler,
                                                             const std::string& variableName,
                                                             const iDynTree::VectorDynSize& maxJointPositionsLimit,
                                                             const iDynTree::VectorDynSize& minJointPositionsLimit,
                                                             const double& samplingTime)
: InequalityConstraintElement(kinDyn)
, m_samplingTime(samplingTime)
, m_minJointPositionsLimit(minJointPositionsLimit)
, m_maxJointPositionsLimit(maxJointPositionsLimit)
{
    m_name = "Joint Values Feasibility Element";

    m_jointAccelerationIndex = handler.getVariable(variableName);

    if (!m_jointAccelerationIndex.isValid())
        throw std::runtime_error("[JointValuesFeasibilityElement::JointValuesFeasibilityElement] "
                                 "Undefined variable named joint_accelerations in the "
                                 "variableHandler");

    // resize and initialize matrices
    m_A.resize(m_jointAccelerationIndex.size, handler.getNumberOfVariables());
    iDynTree::toEigen(m_A).block(0,
                                 m_jointAccelerationIndex.offset,
                                 m_jointAccelerationIndex.size,
                                 m_jointAccelerationIndex.size)
        = Eigen::MatrixXd::Identity(m_jointAccelerationIndex.size, m_jointAccelerationIndex.size)
          * std::pow(samplingTime, 2) / 2;

    m_l.resize(m_jointAccelerationIndex.size);
    m_u.resize(m_jointAccelerationIndex.size);

    m_jointPositions.resize(m_jointAccelerationIndex.size);
    m_jointVelocities.resize(m_jointAccelerationIndex.size);
}

const iDynTree::VectorDynSize& JointValuesFeasibilityElement::getUpperBound()
{
    m_kinDynPtr->getJointPos(m_jointPositions);
    m_kinDynPtr->getJointVel(m_jointVelocities);

    iDynTree::toEigen(m_u) = iDynTree::toEigen(m_maxJointPositionsLimit)
                             - iDynTree::toEigen(m_jointPositions)
                             - iDynTree::toEigen(m_jointVelocities) * m_samplingTime;

    return m_u;
}

const iDynTree::VectorDynSize& JointValuesFeasibilityElement::getLowerBound()
{
    m_kinDynPtr->getJointPos(m_jointPositions);
    m_kinDynPtr->getJointVel(m_jointVelocities);

    iDynTree::toEigen(m_l) = iDynTree::toEigen(m_minJointPositionsLimit)
                             - iDynTree::toEigen(m_jointPositions)
                             - iDynTree::toEigen(m_jointVelocities) * m_samplingTime;

    return m_l;
}
