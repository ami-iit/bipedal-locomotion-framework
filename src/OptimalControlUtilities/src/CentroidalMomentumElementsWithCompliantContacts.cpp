/**
 * @file CentroidalMomentumElementsWithCompliantContacts.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/OptimalControlUtilities/CentroidalMomentumElementsWithCompliantContacts.h>

#include <iDynTree/Model/Model.h>

#include <numeric>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

CentroidalLinearMomentumElementWithCompliantContact::
    CentroidalLinearMomentumElementWithCompliantContact(
        std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
        std::unique_ptr<LinearPD<iDynTree::Vector3>> controller,
        const VariableHandler& handler,
        const std::vector<FrameInContactWithContactModel<std::string, std::string>>& framesInContact)
        : ControlTask(kinDyn)
{
    m_name = "Centroidal Linear Momentum Element [w/ Compliant Contacts]";

    // get the index
    m_jointAccelerationIndex = handler.getVariable("joint_accelerations");
    m_baseAccelerationIndex = handler.getVariable("base_acceleration");

    if (!m_baseAccelerationIndex.isValid())
        throw std::runtime_error("[CentroidalLinearMomentumElementWithCompliantContact::"
                                 "CentroidalLinearMomentumElementWithCompliantContact] Undefined "
                                 "base_acceleration variable");

    if (!m_jointAccelerationIndex.isValid())
        throw std::runtime_error("[CentroidalLinearMomentumElementWithCompliantContact::"
                                 "CentroidalLinearMomentumElementWithCompliantContact] Undefined "
                                 "joint_accelerations variable");

    // get the robot weight expressed in the inertial frame. (The z axis points upwards)
    double gravity = 9.81;
    m_robotWeight.zero();
    m_robotWeight(2) = -gravity * m_kinDynPtr->model().getTotalMass();


    // resize and reset matrices
    m_A.resize(3, handler.getNumberOfVariables());
    m_A.zero();
    m_b.resize(3);
    m_b.zero();

    m_jacobianMatrix.resize(6, m_jointAccelerationIndex.size + m_baseAccelerationIndex.size);

    // initialize the PD
    m_pd = std::move(controller);

    for (const auto& frame : framesInContact)
    {
        const auto& frameNameInVariableHandler = frame.identifierInVariableHandler();
        const auto& frameNameInModel = frame.identifierInModel();
        const auto& contactModel = frame.contactModel();

        if (m_framesInContact.find(frameNameInVariableHandler) != m_framesInContact.end())
        {
            std::cerr << "[CentroidalLinearMomentumElementWithCompliantContact::"
                         "CentroidalLinearMomentumElementWithCompliantContact] The frame named "
                             + frameNameInVariableHandler
                             + " has been already added to the list. The frame will be skipped."
                      << std::endl;
            continue;
        }

        const auto& indexRangeInElement = handler.getVariable(frameNameInVariableHandler);
        const auto& indexInModel = m_kinDynPtr->model().getFrameIndex(frameNameInModel);

        if (!indexRangeInElement.isValid())
            throw std::runtime_error("[CentroidalLinearMomentumElementWithCompliantContact::"
                                     "CentroidalLinearMomentumElementWithCompliantContact] "
                                     "Undefined frame named "
                                     + frameNameInVariableHandler + " in the variableHandler");

        if (indexInModel == iDynTree::FRAME_INVALID_INDEX)
            throw std::runtime_error("[CentroidalLinearMomentumElementWithCompliantContact::"
                                     "CentroidalLinearMomentumElementWithCompliantContact] "
                                     "Undefined frame named "
                                     + frameNameInModel + " in the model");

        if (contactModel == nullptr)
            throw std::runtime_error("[CentroidalLinearMomentumElementWithCompliantContact::"
                                     "CentroidalLinearMomentumElementWithCompliantContact] "
                                     "Undefined contact model for the frame named "
                                     + frameNameInVariableHandler + " in the model");

        m_framesInContact.insert(
            {frameNameInVariableHandler, {indexRangeInElement, indexInModel, contactModel}});
    }
}

const iDynTree::MatrixDynSize& CentroidalLinearMomentumElementWithCompliantContact::getA()
{
    auto A(iDynTree::toEigen(m_A));

    bool isFirstIteration = true;
    for (const auto & frame: m_framesInContact)
    {
        // retrieve the Jacobian matrix
        m_kinDynPtr->getFrameFreeFloatingJacobian(frame.second.identifierInModel(), m_jacobianMatrix);
        const iDynTree::Matrix6x6& controlMatrix = frame.second.contactModel()->getControlMatrix();

        if (!isFirstIteration)
        {
            // add the part related to the base
            A.block(0, m_baseAccelerationIndex.offset, 3, m_baseAccelerationIndex.size)
                += iDynTree::toEigen(controlMatrix).topRows<3>()
                   * iDynTree::toEigen(m_jacobianMatrix).leftCols<6>();

            // add the part related to the joint
            A.block(0, m_jointAccelerationIndex.offset, 3, m_jointAccelerationIndex.size)
                += iDynTree::toEigen(controlMatrix).topRows<3>()
                   * iDynTree::toEigen(m_jacobianMatrix).rightCols(m_jointAccelerationIndex.size);
        } else
        {
            // add the part related to the base
            A.block(0, m_baseAccelerationIndex.offset, 3, m_baseAccelerationIndex.size)
                = iDynTree::toEigen(controlMatrix).topRows<3>()
                  * iDynTree::toEigen(m_jacobianMatrix).leftCols<6>();

            // add the part related to the joint
            A.block(0, m_jointAccelerationIndex.offset, 3, m_jointAccelerationIndex.size)
                = iDynTree::toEigen(controlMatrix).topRows<3>()
                  * iDynTree::toEigen(m_jacobianMatrix).rightCols(m_jointAccelerationIndex.size);

            isFirstIteration = false;
        }
    }
    return m_A;
}

const iDynTree::VectorDynSize& CentroidalLinearMomentumElementWithCompliantContact::getB()
{
    // Get the output of the PD controller
    m_b = m_pd->getControllerOutput();

    auto b(iDynTree::toEigen(m_b));

    // take into account the frames in contact with the environment
    for (const auto& frame : m_framesInContact)
    {
        // if the frame is in contact with the environment is considered in the list
        if (frame.second.isInCompliantContact())
            b -= iDynTree::toEigen(frame.second.contactModel()->getAutonomousDynamics()).head<3>()
                 + iDynTree::toEigen(frame.second.contactModel()->getControlMatrix()).topRows<3>()
                       * iDynTree::toEigen(m_kinDynPtr->getFrameBiasAcc(frame.second.identifierInModel()));
    }
    return m_b;
}

void CentroidalLinearMomentumElementWithCompliantContact::setContactState(
    const std::string& name, bool isInContact, const iDynTree::Transform& nullForceTransform)
{
    auto frame = m_framesInContact.find(name);
    if (frame == m_framesInContact.end())
        throw std::runtime_error("[CentroidalLinearMomentumElementWithCompliantContact::"
                                 "setContactState] The frame named "
                                 + name + " does not exist.");

    frame->second.isInCompliantContact() = isInContact;
    if (frame->second.isInCompliantContact())
    {
        const auto& indexInModel = frame->second.identifierInModel();
        frame->second.contactModel()->setState(
            {{"frame_transform", m_kinDynPtr->getWorldTransform(indexInModel)},
             {"null_force_transform", nullForceTransform},
             {"twist", m_kinDynPtr->getFrameVel(indexInModel)}});
    }
}

void CentroidalLinearMomentumElementWithCompliantContact::setDesiredCentroidalLinearMomentum(
    const iDynTree::Vector3& centroidalLinearMomentumSecondDerivative,
    const iDynTree::Vector3& centroidalLinearMomentumDerivative,
    const iDynTree::Vector3& centroidalLinearMomentum) noexcept
{
    // u = centroidalLinearMomentumSecondDerivative_des
    //    + kd (centroidalLinearMomentumDerivative_des - centroidalLinearMomentumDerivative)
    //    + kp (centroidalLinearMomentum_des - centroidalLinearMomentum)
    m_pd->setDesiredTrajectory(centroidalLinearMomentumSecondDerivative,
                               centroidalLinearMomentumDerivative,
                               centroidalLinearMomentum);
}

void CentroidalLinearMomentumElementWithCompliantContact::setMeasuredContactForces(
    const std::vector<iDynTree::LinearForceVector3>& contactForces)
{
    // u = centroidalLinearMomentumSecondDerivative_des
    //    + kd (centroidalLinearMomentumDerivative_des - centroidalLinearMomentumDerivative)
    //    + kp (centroidalLinearMomentum_des - centroidalLinearMomentum)

    // The centroidalLinearMomentumDerivative is equal to the sum of the contact forces and the
    // weight acting on the system
    auto centroidalLinearMomentumDerivative = m_robotWeight;
    centroidalLinearMomentumDerivative = std::accumulate(contactForces.begin(),
                                                         contactForces.end(),
                                                         centroidalLinearMomentumDerivative);
    m_pd->setFeedback(centroidalLinearMomentumDerivative,
                      m_kinDynPtr->getCentroidalTotalMomentum().getLinearVec3());
}

void CentroidalLinearMomentumElementWithCompliantContact::setGains(const iDynTree::Vector3& kp,
                                                                   const iDynTree::Vector3& kd)
{
    // u = centroidalLinearMomentumSecondDerivative_des
    //    + kd (centroidalLinearMomentumDerivative_des - centroidalLinearMomentumDerivative)
    //    + kp (centroidalLinearMomentum_des - centroidalLinearMomentum)

    m_pd->setGains(kp, kd);
}
