/**
 * @file ContactWrenchFeasibilityElement.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <stdexcept>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/ContactWrenchFeasibilityElement.h>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

GeneralContactWrenchFeasibilityElement::GeneralContactWrenchFeasibilityElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const Frame<std::string, std::string>& frameInContact,
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
    m_name = "Contact Wrench Feasibility Element (Frame in contact: ["
             + frameInContact.identifierInVariableHandler() + ", "
             + frameInContact.identifierInModel() + "])";

    const auto& nameInVariableHandler = frameInContact.identifierInVariableHandler();
    const auto& nameInModel = frameInContact.identifierInModel();

    m_frameInContact.identifierInVariableHandler() = handler.getVariable(nameInVariableHandler);
    m_frameInContact.identifierInModel() = m_kinDynPtr->model().getFrameIndex(nameInModel);

    if (!m_frameInContact.identifierInVariableHandler().isValid())
        throw std::runtime_error("[ContactWrenchFeasibilityElement::"
                                 "ContactWrenchFeasibilityElement] Undefined frame named "
                                 + nameInVariableHandler + " in the variableHandler");

    if (m_frameInContact.identifierInModel() == iDynTree::FRAME_INVALID_INDEX)
        throw std::runtime_error("[ContactWrenchFeasibilityElement::"
                                 "ContactWrenchFeasibilityElement] Undefined frame named "
                                 + nameInModel + " in the model");

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


}

ContactWrenchFeasibilityElement::ContactWrenchFeasibilityElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const Frame<std::string, std::string>& frameInContact,
    const int& numberOfPoints,
    const double& staticFrictionCoefficient,
    const double& torsionalFrictionCoefficient,
    const double& minimalNormalForce,
    const iDynTree::Vector2& footLimitX,
    const iDynTree::Vector2& footLimitY,
    const double& infinity)
    : GeneralContactWrenchFeasibilityElement(kinDyn,
                                             handler,
                                             frameInContact,
                                             numberOfPoints,
                                             staticFrictionCoefficient,
                                             torsionalFrictionCoefficient,
                                             minimalNormalForce,
                                             footLimitX,
                                             footLimitY,
                                             infinity)
{
    unsigned int numberOfEquationsFrictionCone = 4 * (numberOfPoints - 2) + 4;

    // equation used to ensures COP feasibility and unilateral force
    int numberOfEquationsFeasibility = 7;
    int numberOfEquations = numberOfEquationsFrictionCone + numberOfEquationsFeasibility;

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
        = m_kinDynPtr->getWorldTransform(m_frameInContact.identifierInModel()).getRotation().inverse();

    // linear force
    iDynTree::toEigen(m_A).block(0,
                                 m_frameInContact.identifierInVariableHandler().offset,
                                 m_AInBodyFrame.rows(),
                                 3)
        = iDynTree::toEigen(m_AInBodyFrame).leftCols(3) * iDynTree::toEigen(m_rotationMatrix);

    // torque
    iDynTree::toEigen(m_A).block(0,
                                 m_frameInContact.identifierInVariableHandler().offset + 3,
                                 m_AInBodyFrame.rows(),
                                 3)
        = iDynTree::toEigen(m_AInBodyFrame).rightCols(3) * iDynTree::toEigen(m_rotationMatrix);

    return m_A;
}

ContactWrenchRateOfChangeFeasibilityElement::ContactWrenchRateOfChangeFeasibilityElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const Frame<std::string, std::string>& frameInContact,
    const int& numberOfPoints,
    const double& staticFrictionCoefficient,
    const double& torsionalFrictionCoefficient,
    const double& minimalNormalForce,
    const iDynTree::Vector2& footLimitX,
    const iDynTree::Vector2& footLimitY,
    const double& infinity,
    const double & samplingTime)
    : GeneralContactWrenchFeasibilityElement(kinDyn,
                                             handler,
                                             frameInContact,
                                             numberOfPoints,
                                             staticFrictionCoefficient,
                                             torsionalFrictionCoefficient,
                                             minimalNormalForce,
                                             footLimitX,
                                             footLimitY,
                                             infinity),
      m_samplingTime(samplingTime)
{
    unsigned int numberOfEquationsFrictionCone = 4 * (numberOfPoints - 2) + 4;

    // equation used to ensures COP feasibility and unilateral force
    int numberOfEquationsFeasibility = 7;
    int numberOfEquations = numberOfEquationsFrictionCone + numberOfEquationsFeasibility;

    m_upperBoundForce.resize(numberOfEquations);
    m_lowerBoundNormalForce = -m_infinity;

    for (unsigned int i = 0; i < numberOfEquations; i++)
    {
        m_l(i) = -m_infinity;
        if (i != m_nominalForceConstraintIndex)
            m_upperBoundForce(i) = 0;
        else
            m_upperBoundForce(i) = -m_minimalNormalForce;
    }
}

void ContactWrenchRateOfChangeFeasibilityElement::isInContact(bool isInContact)
{
    m_upperBoundForce(m_nominalForceConstraintIndex) = isInContact ? -m_minimalNormalForce : 0;
    m_lowerBoundNormalForce = isInContact ? -m_infinity : 0;
}

const iDynTree::MatrixDynSize& ContactWrenchRateOfChangeFeasibilityElement::getA()
{
    // get the rotation matrix
    m_rotationMatrix
        = m_kinDynPtr->getWorldTransform(m_frameInContact.identifierInModel()).getRotation().inverse();

    iDynTree::Vector3 angularVelocity = m_kinDynPtr->getFrameVel(m_frameInContact.identifierInModel()).getAngularVec3();
    iDynTree::AngularMotionVector3 unitRotation;
    iDynTree::toEigen(unitRotation) = - iDynTree::toEigen(angularVelocity) * m_samplingTime;
    iDynTree::Rotation rotationMatrixNextStep = m_rotationMatrix * unitRotation.exp();


    // linear force
    iDynTree::toEigen(m_A).block(0,
                                 m_frameInContact.identifierInVariableHandler().offset,
                                 m_AInBodyFrame.rows(),
                                 3)
        = iDynTree::toEigen(m_AInBodyFrame).leftCols(3) * iDynTree::toEigen(rotationMatrixNextStep) * m_samplingTime;

    // torque
    iDynTree::toEigen(m_A).block(0,
                                 m_frameInContact.identifierInVariableHandler().offset + 3,
                                 m_AInBodyFrame.rows(),
                                 3)
        = iDynTree::toEigen(m_AInBodyFrame).rightCols(3) * iDynTree::toEigen(rotationMatrixNextStep) * m_samplingTime;

    return m_A;
}

void ContactWrenchRateOfChangeFeasibilityElement::setContactWrench(const iDynTree::Wrench& wrench)
{
    m_contatWrench = wrench;
}

const iDynTree::VectorDynSize& ContactWrenchRateOfChangeFeasibilityElement::getUpperBound()
{
    // get the rotation matrix
    m_rotationMatrix
        = m_kinDynPtr->getWorldTransform(m_frameInContact.identifierInModel()).getRotation().inverse();

    iDynTree::Vector3 angularVelocity = m_kinDynPtr->getFrameVel(m_frameInContact.identifierInModel()).getAngularVec3();
    iDynTree::AngularMotionVector3 unitRotation;
    iDynTree::toEigen(unitRotation) = - iDynTree::toEigen(angularVelocity) * m_samplingTime;
    iDynTree::Rotation rotationMatrixNextStep = m_rotationMatrix * unitRotation.exp();


    iDynTree::toEigen(m_u) = iDynTree::toEigen(m_upperBoundForce)
                             - iDynTree::toEigen(m_AInBodyFrame)
                                   * iDynTree::toEigen(rotationMatrixNextStep * m_contatWrench);
    return m_u;
}

const iDynTree::VectorDynSize& ContactWrenchRateOfChangeFeasibilityElement::getLowerBound()
{
    // get the rotation matrix
    m_rotationMatrix
        = m_kinDynPtr->getWorldTransform(m_frameInContact.identifierInModel()).getRotation().inverse();

    iDynTree::Vector3 angularVelocity = m_kinDynPtr->getFrameVel(m_frameInContact.identifierInModel()).getAngularVec3();
    iDynTree::AngularMotionVector3 unitRotation;
    // the negative sign is because we are interested in the velocity of the inertial frame w.r.t. the base frame
    iDynTree::toEigen(unitRotation) = - iDynTree::toEigen(angularVelocity) * m_samplingTime;
    iDynTree::Rotation rotationMatrixNextStep = m_rotationMatrix * unitRotation.exp();

    m_l(m_nominalForceConstraintIndex)
        = m_lowerBoundNormalForce
          - iDynTree::toEigen(m_AInBodyFrame).row(m_nominalForceConstraintIndex)
                * iDynTree::toEigen(rotationMatrixNextStep * m_contatWrench);
    return m_l;
}
