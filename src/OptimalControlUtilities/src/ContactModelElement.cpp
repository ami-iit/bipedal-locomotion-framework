/**
 * @file ContactModelElement.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/OptimalControlUtilities/ContactModelElement.h>

#include <iDynTree/Model/Model.h>

#include <numeric>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

ContactModelElement::ContactModelElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const FrameInContactWithContactModel<std::string, std::string>& frameInContact)
    : ControlTask(kinDyn)
{
    const auto& frameNameInVariableHandler = frameInContact.identifierInVariableHandler();
    const auto& frameNameInModel = frameInContact.identifierInModel();

    m_name = "Contact Model Element [Label " + frameNameInVariableHandler
             + ", Frame: " + frameNameInModel + "]";

    // get the index
    m_jointAccelerationIndex = handler.getVariable("joint_accelerations");
    m_baseAccelerationIndex = handler.getVariable("base_acceleration");

    if (!m_baseAccelerationIndex.isValid())
        throw std::runtime_error("[ContactModelElement::ContactModelElement] Undefined "
                                 "base_acceleration variable");

    if (!m_jointAccelerationIndex.isValid())
        throw std::runtime_error("[ContactModelElement::ContactModelElement] Undefined "
                                 "joint_accelerations variable");

    // resize and reset matrices
    m_A.resize(6, handler.getNumberOfVariables());
    m_A.zero();
    m_b.resize(6);
    m_b.zero();

    // resize the jacobian matrix
    m_jacobianMatrix.resize(6, m_jointAccelerationIndex.size + m_baseAccelerationIndex.size);

    m_frameInContact.identifierInModel() = m_kinDynPtr->model().getFrameIndex(frameNameInModel);
    m_frameInContact.identifierInVariableHandler() = handler.getVariable(frameNameInVariableHandler);
    m_frameInContact.contactModel() = frameInContact.contactModel();

    if (m_frameInContact.identifierInModel() == iDynTree::FRAME_INVALID_INDEX)
        throw std::runtime_error("[ContactModelElement::ContactModelElement] Undefined frame named "
                                 + frameNameInModel + " in the model");

    if (m_frameInContact.contactModel() == nullptr)
        throw std::runtime_error("[ContactModelElement::ContactModelElement] Undefined contact "
                                 "model for the frame named "
                                 + frameNameInVariableHandler + " in the model");

    // Set constant part of the A  matrix
    // A = [********* -I ****] The identity is related to the frame in contact with the environment
    size_t identitySize = m_frameInContact.identifierInVariableHandler().size;
    iDynTree::VectorDynSize onesVector(identitySize);
    for (auto& element : onesVector)
        element = -1;
    iDynTree::toEigen(m_A).block(0,
                                 m_frameInContact.identifierInVariableHandler().offset,
                                 identitySize,
                                 identitySize)
        = iDynTree::toEigen(onesVector).asDiagonal();
}

const iDynTree::MatrixDynSize& ContactModelElement::getA()
{
    using iDynTree::toEigen;

    auto A(toEigen(m_A));

    // retrieve the Jacobian matrix
    m_kinDynPtr->getFrameFreeFloatingJacobian(m_frameInContact.identifierInModel(),
                                              m_jacobianMatrix);

    // get the control matrix associated to the contact model
    const iDynTree::Matrix6x6& controlMatrix = m_frameInContact.contactModel()->getControlMatrix();

    // add the part related to the base
    A.middleCols(m_baseAccelerationIndex.offset, m_baseAccelerationIndex.size)
        = toEigen(controlMatrix) * toEigen(m_jacobianMatrix).leftCols<6>();

    // add the part related to the joint
    A.middleCols(m_jointAccelerationIndex.offset, m_jointAccelerationIndex.size)
        = toEigen(controlMatrix)
          * toEigen(m_jacobianMatrix).rightCols(m_jointAccelerationIndex.size);

    return m_A;
}

const iDynTree::VectorDynSize& ContactModelElement::getB()
{
    using iDynTree::toEigen;

    auto b(toEigen(m_b));
    b = -toEigen(m_frameInContact.contactModel()->getAutonomousDynamics())
        - toEigen(m_frameInContact.contactModel()->getControlMatrix())
              * toEigen(m_kinDynPtr->getFrameBiasAcc(m_frameInContact.identifierInModel()));
    return m_b;
}

void ContactModelElement::setContactState(bool isInContact,
                                          const iDynTree::Transform& nullForceTransform)
{
    m_frameInContact.isInCompliantContact() = isInContact;
    const auto& indexInModel = m_frameInContact.identifierInModel();
    m_frameInContact.contactModel()->setState(m_kinDynPtr->getFrameVel(indexInModel),
                                              m_kinDynPtr->getWorldTransform(indexInModel),
                                              nullForceTransform);
}
