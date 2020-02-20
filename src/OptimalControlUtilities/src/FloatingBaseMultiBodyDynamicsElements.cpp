/**
 * @file FloatingBaseMultiBodyDynamicsElements.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <stdexcept>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/FloatingBaseMultiBodyDynamicsElements.h>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

// MultiBodyDynamicsElement
MultiBodyDynamicsElement::MultiBodyDynamicsElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const std::vector<FrameInContact<std::string, std::string>>& framesInContact)
    : ControlTask(kinDyn)
{
    m_name = "Multi Body Dynamics Element";

    m_jointAccelerationIndex = handler.getVariable("joint_accelerations");
    m_baseAccelerationIndex = handler.getVariable("base_acceleration");


    if (!m_baseAccelerationIndex.isValid())
        throw std::runtime_error("[MultiBodyDynamicsElement::"
                                 "MultiBodyDynamicsElement] Undefined "
                                 "base_acceleration variable");

    if (!m_jointAccelerationIndex.isValid())
        throw std::runtime_error("[MultiBodyDynamicsElement::"
                                 "MultiBodyDynamicsElement] Undefined "
                                 "joint_accelerations variable");


    for (const auto& frame : framesInContact)
    {
        const auto& indexRangeInElement = handler.getVariable(frame.identifierInVariableHandler());
        const auto& indexInModel = m_kinDynPtr->model().getFrameIndex(frame.identifierInModel());
        bool isInCompliantContact = frame.isInCompliantContact();

        // If the contact between the environment and the robot is described using a compliant model
        // the contact force is not one of the unknown parameters. So it will not be considered in
        // the VariableHandler. If the variable is not defined an invalid IndexRange is provided
        if (!isInCompliantContact && !indexRangeInElement.isValid())
            throw std::runtime_error("[MultiBodyDynamicsElement::MultiBodyDynamicsElement] "
                                     "Undefined frame named "
                                     + frame.identifierInVariableHandler() + " in the variableHandler");

        if (indexInModel == iDynTree::FRAME_INVALID_INDEX)
            throw std::runtime_error("[MultiBodyDynamicsElement::MultiBodyDynamicsElement] "
                                     "Undefined frame named "
                                     + frame.identifierInModel() + " in the model");

        if (m_framesInContact.find(frame.identifierInModel()) != m_framesInContact.end())
            throw std::runtime_error("[MultiBodyDynamicsElement::MultiBodyDynamicsElement] "
                                     "The frame named "
                                     + frame.identifierInModel()
                                     + " has been already added to the list of the frames in "
                                     "contact");

        m_framesInContact.insert({frame.identifierInVariableHandler(),
                                  {indexRangeInElement, indexInModel, isInCompliantContact}});
    }

    // resize quantities related to the system dynamics
    // the degrees of freedom are equal to the robot joint plus the base
    m_jacobianMatrix.resize(6, m_baseAccelerationIndex.size + m_jointAccelerationIndex.size);
    m_massMatrix.resize(m_baseAccelerationIndex.size + m_jointAccelerationIndex.size,
                        m_baseAccelerationIndex.size + m_jointAccelerationIndex.size);
    m_generalizedBiasForces.resize(m_kinDynPtr->model());
}

void MultiBodyDynamicsElement::setExternalWrench(const std::string& frameName,
                                                 const iDynTree::Wrench& wrench)
{
    auto frameInContact = m_framesInContact.find(frameName);
    if (frameInContact == m_framesInContact.end())
        throw std::runtime_error("[ MultiBodyDynamicsElement::setExternalWrench] The frame named "
                                 + frameName + " is not one of the frame considered in contact");

    frameInContact->second.contactWrench() = wrench;
}

void MultiBodyDynamicsElement::setCompliantContact(const std::string& frameName, bool isCompliant)
{
    auto frameInContact = m_framesInContact.find(frameName);
    if (frameInContact == m_framesInContact.end())
        throw std::runtime_error("[ MultiBodyDynamicsElement::setExternalWrench] The frame named "
                                 + frameName + " is not one of the frame considered in contact");

    frameInContact->second.isInCompliantContact() = isCompliant;
}

// Floating Base dynamical system
FloatingBaseDynamicsElement::FloatingBaseDynamicsElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const std::vector<FrameInContact<std::string, std::string>>& framesInContact)
    : MultiBodyDynamicsElement(kinDyn, handler, framesInContact)
{
    m_name += " [Base dynamics]";

    // resize A and b
    m_A.resize(m_baseAccelerationIndex.size, handler.getNumberOfVariables());
    m_A.zero();

    m_b.resize(m_baseAccelerationIndex.size);
    m_b.zero();
}

const iDynTree::MatrixDynSize& FloatingBaseDynamicsElement::getA()
{
    using iDynTree::toEigen;

    // store the massMatrix
    m_kinDynPtr->getFreeFloatingMassMatrix(m_massMatrix);

    // M =  [M_bb   M_bs
    //       M_sb   M_ss]

    // Since this element considers only the base dynamics only the M_bb and M_bs blocks will be
    // considered

    // M_bb
    toEigen(m_A).middleCols(m_baseAccelerationIndex.offset, m_baseAccelerationIndex.size)
        = -toEigen(m_massMatrix)
               .topLeftCorner(m_baseAccelerationIndex.size, m_baseAccelerationIndex.size);

    // M_bs
    toEigen(m_A).middleCols(m_jointAccelerationIndex.offset, m_jointAccelerationIndex.size)
        = -toEigen(m_massMatrix)
               .topRightCorner(m_baseAccelerationIndex.size, m_jointAccelerationIndex.size);

    // store the jacobians
    for (const auto& frameInContact : m_framesInContact)
    {
        const auto& frame = frameInContact.second;

        // if the contact between the link associated to the frame and the environment is not
        // modelled as compliant (i.e. stiff contact) the desired contact wrench is an unknown
        // variable. So the Jacobian matrix is stored inside the matrix A
        if (!frame.isInCompliantContact())
        {
            m_kinDynPtr->getFrameFreeFloatingJacobian(frame.identifierInModel(), m_jacobianMatrix);

            // copy only the frame associated to the base (first 6 rows)
            toEigen(m_A).middleCols(frame.identifierInVariableHandler().offset,
                                    frame.identifierInVariableHandler().size)
                = toEigen(m_jacobianMatrix).leftCols(m_baseAccelerationIndex.size).transpose();
        }
    }

    return m_A;
}

const iDynTree::VectorDynSize& FloatingBaseDynamicsElement::getB()
{
    using iDynTree::toEigen;

    m_kinDynPtr->generalizedBiasForces(m_generalizedBiasForces);
    toEigen(m_b) = toEigen(m_generalizedBiasForces.baseWrench());

    // Compute the wrenches
    for (const auto& frameInContact : m_framesInContact)
    {
        const auto& frame = frameInContact.second;

        // if the contact between the link associated to the frame and the environment is
        // modelled as compliant the desired contact wrench is not an unknown
        // variable.
        if (frame.isInCompliantContact())
        {
            m_kinDynPtr->getFrameFreeFloatingJacobian(frame.identifierInModel(), m_jacobianMatrix);

            // copy only the frame associated to the base (first 6 rows)
            toEigen(m_b)
                -= toEigen(m_jacobianMatrix).leftCols(m_baseAccelerationIndex.size).transpose()
                   * toEigen(frame.contactWrench());
        }
    }

    return m_b;
}

// Joint space dynamical system
JointSpaceDynamicsElement::JointSpaceDynamicsElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const std::vector<FrameInContact<std::string, std::string>>& framesInContact)
    : MultiBodyDynamicsElement(kinDyn, handler, framesInContact)
{
    m_name += " [Joint dynamics]";
    m_jointTorqueIndex = handler.getVariable("joint_torques");

    if (!m_jointTorqueIndex.isValid())
        throw std::runtime_error("[JointSpaceDynamicsElement::JointSpaceDynamicsElement Undefined "
                                 "joint_torques variable");

    // resize A and b
    m_A.resize(m_jointAccelerationIndex.size, handler.getNumberOfVariables());
    m_A.zero();

    m_b.resize(m_jointAccelerationIndex.size);
    m_b.zero();

    // set constant value of m_A
    // the part related to the joint torques is the Identity matrix
    iDynTree::toEigen(m_A)
        .block(0, m_jointTorqueIndex.offset, m_jointTorqueIndex.size, m_jointTorqueIndex.size)
        .setIdentity();
}

JointSpaceDynamicsElement::JointSpaceDynamicsElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const std::vector<FrameInContact<std::string, std::string>>& framesInContact,
    const iDynTree::MatrixDynSize& regularizationMatrix)
    : JointSpaceDynamicsElement(kinDyn, handler, framesInContact)
{
    m_name += " (with regularization matrix)";
    m_useReflectedInertia = true;

    unsigned int actuatedDoFs = m_jointTorqueIndex.size;
    if (regularizationMatrix.rows() != actuatedDoFs)
        throw std::runtime_error("[JointSpaceDynamicsElement::JointSpaceDynamicsElement] The "
                                 "number of rows of the regularizationMatrix is not equal to the "
                                 "one expected.  Expected: "
                                 + std::to_string(actuatedDoFs)
                                 + " retrieved: " + std::to_string(regularizationMatrix.rows()));

    if (regularizationMatrix.cols() != actuatedDoFs)
        throw std::runtime_error("[JointSpaceDynamicsElement::JointSpaceDynamicsElement] The "
                                 "number of columns of the regularizationMatrix is not equal to the "
                                 "one expected.  Expected: "
                                 + std::to_string(actuatedDoFs)
                                 + " retrieved: " + std::to_string(regularizationMatrix.rows()));

    m_reflectedInertia = regularizationMatrix;
}

const iDynTree::MatrixDynSize& JointSpaceDynamicsElement::getA()
{

    using iDynTree::toEigen;

    // store the massMatrix
    m_kinDynPtr->getFreeFloatingMassMatrix(m_massMatrix);

    // M =  [M_bb   M_bs
    //       M_sb   M_ss]

    // Since this element considers only the base dynamics only the M_sb and M_ss blocks will be
    // considered

    // M_sb
    toEigen(m_A).middleCols(m_baseAccelerationIndex.offset, m_baseAccelerationIndex.size)
        = -toEigen(m_massMatrix)
               .bottomLeftCorner(m_jointAccelerationIndex.size, m_baseAccelerationIndex.size);

    // M_ss
    if (!m_useReflectedInertia)
        toEigen(m_A).middleCols(m_jointAccelerationIndex.offset, m_jointAccelerationIndex.size)
            = -toEigen(m_massMatrix)
                   .bottomRightCorner(m_jointAccelerationIndex.size, m_jointAccelerationIndex.size);
    else
        toEigen(m_A).middleCols(m_jointAccelerationIndex.offset, m_jointAccelerationIndex.size)
            = -(toEigen(m_massMatrix)
                    .bottomRightCorner(m_jointAccelerationIndex.size, m_jointAccelerationIndex.size)
                + toEigen(m_reflectedInertia));

    // store the jacobians
    for (const auto& frameInContact : m_framesInContact)
    {
        const auto& frame = frameInContact.second;

        // if the contact between the link associated to the frame and the environment is not
        // modelled as compliant (i.e. stiff contact) the desired contact wrench is an unknown
        // variable. So the Jacobian matrix is stored inside the matrix A
        if (!frame.isInCompliantContact())
        {
            m_kinDynPtr->getFrameFreeFloatingJacobian(frame.identifierInModel(), m_jacobianMatrix);

            // copy only the frame associated to the joint (last "actuated DoFs" rows)
            toEigen(m_A).middleCols(frame.identifierInVariableHandler().offset,
                                    frame.identifierInVariableHandler().size)
                = toEigen(m_jacobianMatrix).rightCols(m_jointAccelerationIndex.size).transpose();
        }
    }
    return m_A;
}

const iDynTree::VectorDynSize& JointSpaceDynamicsElement::getB()
{
    using iDynTree::toEigen;

    m_kinDynPtr->generalizedBiasForces(m_generalizedBiasForces);
    m_b = m_generalizedBiasForces.jointTorques();

    // store the jacobians
    for (const auto& frameInContact : m_framesInContact)
    {
        const auto& frame = frameInContact.second;

        // if the contact between the link associated to the frame and the environment is
        // modelled as compliant the desired contact wrench is not an unknown
        // variable.
        if (frame.isInCompliantContact())
        {
            m_kinDynPtr->getFrameFreeFloatingJacobian(frame.identifierInModel(), m_jacobianMatrix);

            // copy only the frame associated to the joint (last "actuated DoFs" rows)
            toEigen(m_b)
                -= toEigen(m_jacobianMatrix).rightCols(m_jointAccelerationIndex.size).transpose()
                   * toEigen(frame.contactWrench());
        }
    }

    return m_b;
}

// Whole Body Floating Base Dynamics Element
WholeBodyFloatingBaseDynamicsElement::WholeBodyFloatingBaseDynamicsElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const std::vector<FrameInContact<std::string, std::string>>& framesInContact)
    : MultiBodyDynamicsElement(kinDyn, handler, framesInContact)
{
    m_name += " [Whole body dynamics]";
    m_jointTorqueIndex = handler.getVariable("joint_torques");

    if (!m_jointTorqueIndex.isValid())
        throw std::runtime_error("[JointSpaceDynamicsElement::JointSpaceDynamicsElement Undefined "
                                 "joint_torques variable");

    m_A.resize(m_baseAccelerationIndex.size + m_jointAccelerationIndex.size,
               handler.getNumberOfVariables());
    m_A.zero();

    m_b.resize(m_baseAccelerationIndex.size + m_jointAccelerationIndex.size);
    m_b.zero();

    // set constant value of m_A
    // the part related to the joint torques is [0;I]
    iDynTree::toEigen(m_A)
        .block(6, m_jointTorqueIndex.offset, m_jointTorqueIndex.size, m_jointTorqueIndex.size)
        .setIdentity();
}

WholeBodyFloatingBaseDynamicsElement::WholeBodyFloatingBaseDynamicsElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const std::vector<FrameInContact<std::string, std::string>>& framesInContact,
    const iDynTree::MatrixDynSize& regularizationMatrix)
    : WholeBodyFloatingBaseDynamicsElement(kinDyn, handler, framesInContact)
{
    m_name += " (with regularization matrix)";
    m_useReflectedInertia = true;

    unsigned int actuatedDoFs = m_jointTorqueIndex.size;
    if (regularizationMatrix.rows() != actuatedDoFs)
        throw std::runtime_error("[JointSpaceDynamicsElement::JointSpaceDynamicsElement] The "
                                 "number of rows of the regularizationMatrix is not equal to the "
                                 "one expected.  Expected: "
                                 + std::to_string(actuatedDoFs)
                                 + " retrieved: " + std::to_string(regularizationMatrix.rows()));

    if (regularizationMatrix.cols() != actuatedDoFs)
        throw std::runtime_error("[JointSpaceDynamicsElement::JointSpaceDynamicsElement] The "
                                 "number of columns of the regularizationMatrix is not equal to the "
                                 "one expected.  Expected: "
                                 + std::to_string(actuatedDoFs)
                                 + " retrieved: " + std::to_string(regularizationMatrix.rows()));

    m_reflectedInertia = regularizationMatrix;
}

const iDynTree::MatrixDynSize& WholeBodyFloatingBaseDynamicsElement::getA()
{
    using iDynTree::toEigen;

    // store the massMatrix
    m_kinDynPtr->getFreeFloatingMassMatrix(m_massMatrix);

    // M =  [M_bb   M_bs
    //       M_sb   M_ss]
    // M_bb
    toEigen(m_A).block(0,
                       m_baseAccelerationIndex.offset,
                       m_baseAccelerationIndex.size,
                       m_baseAccelerationIndex.size)
        = -toEigen(m_massMatrix)
               .topLeftCorner(m_baseAccelerationIndex.size, m_baseAccelerationIndex.size);

    // M_bs
    toEigen(m_A).block(0,
                       m_jointAccelerationIndex.offset,
                       m_baseAccelerationIndex.size,
                       m_jointAccelerationIndex.size)
        = -toEigen(m_massMatrix)
               .topRightCorner(m_baseAccelerationIndex.size, m_jointAccelerationIndex.size);

    // M_sb
    toEigen(m_A).block(m_baseAccelerationIndex.size,
                       m_baseAccelerationIndex.offset,
                       m_jointAccelerationIndex.size,
                       m_baseAccelerationIndex.size)
        = -toEigen(m_massMatrix)
               .bottomLeftCorner(m_jointAccelerationIndex.size, m_baseAccelerationIndex.size);

    // M_ss
    if (!m_useReflectedInertia)
        toEigen(m_A).block(m_baseAccelerationIndex.size,
                           m_jointAccelerationIndex.offset,
                           m_jointAccelerationIndex.size,
                           m_jointAccelerationIndex.size)
            = -toEigen(m_massMatrix)
                   .bottomRightCorner(m_jointAccelerationIndex.size, m_jointAccelerationIndex.size);
    else
        toEigen(m_A).block(m_baseAccelerationIndex.size,
                           m_jointAccelerationIndex.offset,
                           m_jointAccelerationIndex.size,
                           m_jointAccelerationIndex.size)
            = -(toEigen(m_massMatrix)
                    .bottomRightCorner(m_jointAccelerationIndex.size, m_jointAccelerationIndex.size)
                + toEigen(m_reflectedInertia));

    // store the jacobians
    for (const auto& frameInContact : m_framesInContact)
    {
        const auto& frame = frameInContact.second;

        // if the contact between the link associated to the frame and the environment is not
        // modelled as compliant (i.e. stiff contact) the desired contact wrench is an unknown
        // variable. So the Jacobian matrix is stored inside the matrix A
        if (!frame.isInCompliantContact())
        {
            m_kinDynPtr->getFrameFreeFloatingJacobian(frame.identifierInModel(), m_jacobianMatrix);

            toEigen(m_A).block(0,
                               frame.identifierInVariableHandler().offset,
                               m_jointAccelerationIndex.size + m_baseAccelerationIndex.size,
                               frame.identifierInVariableHandler().size)
                = toEigen(m_jacobianMatrix).transpose();
        }
    }

    return m_A;
}

const iDynTree::VectorDynSize& WholeBodyFloatingBaseDynamicsElement::getB()
{
    using iDynTree::toEigen;

    m_kinDynPtr->generalizedBiasForces(m_generalizedBiasForces);
    toEigen(m_b).head(m_baseAccelerationIndex.size) = toEigen(m_generalizedBiasForces.baseWrench());
    toEigen(m_b).tail(m_jointAccelerationIndex.size) = toEigen(m_generalizedBiasForces.jointTorques());

    // Compute the wrenches
    for (const auto& frameInContact : m_framesInContact)
    {
        const auto& frame = frameInContact.second;

        // if the contact between the link associated to the frame and the environment is
        // modelled as compliant the desired contact wrench is not an unknown
        // variable.
        if (frame.isInCompliantContact())
        {
            m_kinDynPtr->getFrameFreeFloatingJacobian(frame.identifierInModel(), m_jacobianMatrix);
            toEigen(m_b) -= toEigen(m_jacobianMatrix).transpose() * toEigen(frame.contactWrench());
        }
    }
    return m_b;
}
