/**
 * @file FeasibilityElement.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <stdexcept>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/FeasibilityElements.h>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

ContactWrenchFeasibilityElement::ContactWrenchFeasibilityElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                                                 const VariableHandler& handler,
                                                                 const std::pair<std::string, std::string>& frameInContact,
                                                                 const int& numberOfPoints,
                                                                 const double& staticFrictionCoefficient,
                                                                 const double& torsionalFrictionCoefficient,
                                                                 const double& minimalNormalForce,
                                                                 const iDynTree::Vector2& footLimitX,
                                                                 const iDynTree::Vector2& footLimitY,
                                                                 const double& infinity)
: InequalityConstraintElement(kinDyn)
, m_infinity(infinity)
, m_minimalNormalForce(minimalNormalForce)
{
    m_name = "Contact Wrench Feasibility Element (Frame in contact: [" + frameInContact.first
             + ",  " + frameInContact.second + "])";

    m_frameInContact.indexRangeInElement = handler.getVariable(frameInContact.first);
    m_frameInContact.indexInModel = m_kinDynPtr->model().getFrameIndex(frameInContact.second);

    if (!m_frameInContact.indexRangeInElement.isValid())
        throw std::runtime_error("[ContactWrenchFeasibilityElement::"
                                 "ContactWrenchFeasibilityElement] Undefined frame named "
                                 + frameInContact.first + " in the variableHandler");

    if (m_frameInContact.indexInModel == iDynTree::FRAME_INVALID_INDEX)
        throw std::runtime_error("[ContactWrenchFeasibilityElement::"
                                 "ContactWrenchFeasibilityElement] Undefined frame named "
                                 + frameInContact.second + " in the model");

    // split the friction cone into slices
    double segmentAngle = iDynTree::deg2rad(90) / (numberOfPoints - 1);
    unsigned int numberOfEquationsFrictionCone = 4 * (numberOfPoints - 2) + 4;
    m_nominalForceConstraintIndex = numberOfEquationsFrictionCone + 2;

    // equation used to ensures COP feasibility and unilateral force
    int numberOfEquationsFeasibility = 7;
    int numberOfEquations = numberOfEquationsFrictionCone + numberOfEquationsFeasibility;

    m_AInBodyFrame.resize(numberOfEquations, 6);
    m_AInBodyFrame.zero();

    // resize and clear the matrices
    m_u.resize(numberOfEquations);
    m_l.resize(numberOfEquations);
    m_A.resize(numberOfEquations, handler.getNumberOfVariables());
    m_u.zero();
    m_l.zero();
    m_A.zero();

    // evaluate friction cone constraint
    iDynTree::VectorDynSize angles(numberOfEquationsFrictionCone);
    iDynTree::VectorDynSize pointsX(numberOfEquationsFrictionCone);
    iDynTree::VectorDynSize pointsY(numberOfEquationsFrictionCone);

    for (unsigned int i = 0; i < numberOfEquationsFrictionCone; i++)
    {
        angles(i) = i * segmentAngle;
        pointsX(i) = cos(angles(i));
        pointsY(i) = sin(angles(i));
    }

    for (unsigned int i = 0; i < numberOfEquationsFrictionCone; i++)
    {
        double firstPointX, firstPointY, secondPointX, secondPointY;
        firstPointX = pointsX(i);
        firstPointY = pointsY(i);

        secondPointX = pointsX((i + 1) % numberOfEquationsFrictionCone);
        secondPointY = pointsY((i + 1) % numberOfEquationsFrictionCone);

        double angularCoefficients;
        angularCoefficients = (secondPointY - firstPointY) / (secondPointX - firstPointX);

        double offset;
        offset = firstPointY - angularCoefficients * firstPointX;

        int inequalityFactor = 1;
        if (angles(i) > iDynTree::deg2rad(180)
            || angles((i + 1) % numberOfEquationsFrictionCone) > iDynTree::deg2rad(180))
            inequalityFactor = -1;

        //  A_ineq(i,:) = inequalityFactor.* [-angularCoefficients, 1,
        //  (-offsets*staticFrictionCoefficient), 0, 0, 0];
        m_AInBodyFrame(i, 0) = -inequalityFactor * angularCoefficients;
        m_AInBodyFrame(i, 1) = inequalityFactor;
        m_AInBodyFrame(i, 2) = -inequalityFactor * offset * staticFrictionCoefficient;
    }

    // Unilateral constraint and COP position
    m_AInBodyFrame(numberOfEquationsFrictionCone, 2) = -torsionalFrictionCoefficient;
    m_AInBodyFrame(numberOfEquationsFrictionCone + 1, 2) = -torsionalFrictionCoefficient;
    m_AInBodyFrame(numberOfEquationsFrictionCone + 2, 2) = -1;
    m_AInBodyFrame(numberOfEquationsFrictionCone + 3, 2) = footLimitX(0);
    m_AInBodyFrame(numberOfEquationsFrictionCone + 4, 2) = -footLimitX(1);
    m_AInBodyFrame(numberOfEquationsFrictionCone + 5, 2) = footLimitY(0);
    m_AInBodyFrame(numberOfEquationsFrictionCone + 6, 2) = -footLimitY(1);

    m_AInBodyFrame(numberOfEquationsFrictionCone + 5, 3) = -1;
    m_AInBodyFrame(numberOfEquationsFrictionCone + 6, 3) = 1;

    m_AInBodyFrame(numberOfEquationsFrictionCone + 3, 4) = 1;
    m_AInBodyFrame(numberOfEquationsFrictionCone + 4, 4) = -1;

    m_AInBodyFrame(numberOfEquationsFrictionCone, 5) = 1;
    m_AInBodyFrame(numberOfEquationsFrictionCone + 1, 5) = -1;

    for (unsigned int i = 0; i < numberOfEquations; i++)
    {
        m_l(i) = -m_infinity;
        if (i != m_nominalForceConstraintIndex)
            m_u(i) = 0;
        else
            m_u(i) = -m_minimalNormalForce;
    }
}

void ContactWrenchFeasibilityElement::isInContact(bool isInContact)
{
    m_u(m_nominalForceConstraintIndex) = isInContact ? -m_minimalNormalForce : 0;
    m_l(m_nominalForceConstraintIndex) = isInContact ? -m_infinity : 0;
}

const iDynTree::MatrixDynSize& ContactWrenchFeasibilityElement::getA()
{
    // get the rotation matrix
    m_rotationMatrix
        = m_kinDynPtr->getWorldTransform(m_frameInContact.indexInModel).getRotation().inverse();

    // linear force
    iDynTree::toEigen(m_A).block(0,
                                 m_frameInContact.indexRangeInElement.offset,
                                 m_AInBodyFrame.rows(),
                                 3)
        = iDynTree::toEigen(m_AInBodyFrame).leftCols(3) * iDynTree::toEigen(m_rotationMatrix);

    // torque
    iDynTree::toEigen(m_A).block(0,
                                 m_frameInContact.indexRangeInElement.offset + 3,
                                 m_AInBodyFrame.rows(),
                                 3)
        = iDynTree::toEigen(m_AInBodyFrame).rightCols(3) * iDynTree::toEigen(m_rotationMatrix);

    return m_A;
}
const iDynTree::VectorDynSize& ContactWrenchFeasibilityElement::getUpperBound()
{
    return m_u;
}

const iDynTree::VectorDynSize& ContactWrenchFeasibilityElement::getLowerBound()
{
    return m_l;
}

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
