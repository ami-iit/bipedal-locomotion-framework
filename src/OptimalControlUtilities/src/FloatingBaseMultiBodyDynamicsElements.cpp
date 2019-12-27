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

// System Dynamics
FloatingBaseMultiBodyDynamicsElement::FloatingBaseMultiBodyDynamicsElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const std::vector<FrameNames>& framesInContact)
    : ControlTask(kinDyn)
{
    // if this constructor is called the motor reflected inertia will not be used
    m_useReflectedInertia = false;

    m_jointAccelerationIndex = handler.getVariable("joint_accelerations");
    m_baseAccelerationIndex = handler.getVariable("base_acceleration");
    m_jointTorqueIndex = handler.getVariable("joint_torques");

    m_name = "System Dynamics Element";

    if (!m_baseAccelerationIndex.isValid())
        throw std::runtime_error("[FloatingBaseMultiBodyDynamicsElement::"
                                 "FloatingBaseMultiBodyDynamicsElement] Undefined "
                                 "base_acceleration variable");

    if (!m_jointAccelerationIndex.isValid())
        throw std::runtime_error("[FloatingBaseMultiBodyDynamicsElement::"
                                 "FloatingBaseMultiBodyDynamicsElement] Undefined "
                                 "joint_accelerations variable");

    if (!m_jointTorqueIndex.isValid())
        throw std::runtime_error("[FloatingBaseMultiBodyDynamicsElement::"
                                 "FloatingBaseMultiBodyDynamicsElement] Undefined "
                                 "joint_torques variable");

    for (const auto& frame : framesInContact)
    {
        Frame frameInContact;
        frameInContact.indexRangeInElement = handler.getVariable(frame.label());
        frameInContact.indexInModel = m_kinDynPtr->model().getFrameIndex(frame.nameInModel());

        if (!frameInContact.indexRangeInElement.isValid())
            throw std::runtime_error("[FloatingBaseMultiBodyDynamicsElement::"
                                     "FloatingBaseMultiBodyDynamicsElement] Undefined "
                                     "frame named "
                                     + frame.label() + " in the variableHandler");

        if (frameInContact.indexInModel == iDynTree::FRAME_INVALID_INDEX)
            throw std::runtime_error("[FloatingBaseMultiBodyDynamicsElement::"
                                     "FloatingBaseMultiBodyDynamicsElement] Undefined "
                                     "frame named "
                                     + frame.nameInModel() + " in the model");

        m_framesInContact.push_back(frameInContact);
    }
    m_jacobianMatrix.resize(6, m_baseAccelerationIndex.size + m_jointAccelerationIndex.size);

    m_massMatrix.resize(m_baseAccelerationIndex.size + m_jointAccelerationIndex.size,
                        m_baseAccelerationIndex.size + m_jointAccelerationIndex.size);

    // resize A and b
    m_A.resize(m_baseAccelerationIndex.size + m_jointAccelerationIndex.size,
               handler.getNumberOfVariables());
    m_A.zero();

    m_b.resize(m_baseAccelerationIndex.size + m_jointAccelerationIndex.size);
    m_b.zero();

    // resize generalizedBiasForces
    m_generalizedBiasForces.resize(m_kinDynPtr->model());

    // set constant value of m_A
    // the part related to the joint torques is [0;I]
    iDynTree::toEigen(m_A)
        .block(6, m_jointTorqueIndex.offset, m_jointTorqueIndex.size, m_jointTorqueIndex.size)
        .setIdentity();
}

FloatingBaseMultiBodyDynamicsElement::FloatingBaseMultiBodyDynamicsElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const std::vector<FrameNames>& framesInContact,
    const iDynTree::MatrixDynSize& regularizationMatrix)
    : FloatingBaseMultiBodyDynamicsElement(kinDyn, handler, framesInContact)
{
    m_useReflectedInertia = true;
    m_name += " (with regularization matrix)";

    unsigned int actuatedDoFs = m_jointTorqueIndex.size;
    if (regularizationMatrix.rows() != actuatedDoFs)
        throw std::runtime_error("[FloatingBaseMultiBodyDynamicsElement::"
                                 "FloatingBaseMultiBodyDynamicsElement] The number of "
                                 "rows of the regularizationMatrix is not equal to the one "
                                 "expected.  Expected: "
                                 + std::to_string(actuatedDoFs)
                                 + " retrieved: " + std::to_string(regularizationMatrix.rows()));

    if (regularizationMatrix.cols() != actuatedDoFs)
        throw std::runtime_error("[FloatingBaseMultiBodyDynamicsElement::"
                                 "FloatingBaseMultiBodyDynamicsElement] The number of "
                                 "columns of the regularizationMatrix is not equal to the one "
                                 "expected.  Expected: "
                                 + std::to_string(actuatedDoFs)
                                 + " retrieved: " + std::to_string(regularizationMatrix.cols()));

    m_reflectedInertia = regularizationMatrix;
}

const iDynTree::MatrixDynSize& FloatingBaseMultiBodyDynamicsElement::getA()
{
    // store the massMatrix
    m_kinDynPtr->getFreeFloatingMassMatrix(m_massMatrix);

    // M =  [M_bb   M_bs
    //       M_sb   M_ss]
    // M_bb
    iDynTree::toEigen(m_A).block(0,
                                 m_baseAccelerationIndex.offset,
                                 m_baseAccelerationIndex.size,
                                 m_baseAccelerationIndex.size)
        = -iDynTree::toEigen(m_massMatrix)
               .topLeftCorner(m_baseAccelerationIndex.size, m_baseAccelerationIndex.size);

    // M_bs
    iDynTree::toEigen(m_A).block(0,
                                 m_jointAccelerationIndex.offset,
                                 m_baseAccelerationIndex.size,
                                 m_jointAccelerationIndex.size)
        = -iDynTree::toEigen(m_massMatrix)
               .topRightCorner(m_baseAccelerationIndex.size, m_jointAccelerationIndex.size);

    // M_sb
    iDynTree::toEigen(m_A).block(m_baseAccelerationIndex.size,
                                 m_baseAccelerationIndex.offset,
                                 m_jointAccelerationIndex.size,
                                 m_baseAccelerationIndex.size)
        = -iDynTree::toEigen(m_massMatrix)
               .bottomLeftCorner(m_jointAccelerationIndex.size, m_baseAccelerationIndex.size);

    // M_ss
    if (!m_useReflectedInertia)
        iDynTree::toEigen(m_A).block(m_baseAccelerationIndex.size,
                                     m_jointAccelerationIndex.offset,
                                     m_jointAccelerationIndex.size,
                                     m_jointAccelerationIndex.size)
            = -iDynTree::toEigen(m_massMatrix)
                   .bottomRightCorner(m_jointAccelerationIndex.size, m_jointAccelerationIndex.size);
    else
        iDynTree::toEigen(m_A).block(m_baseAccelerationIndex.size,
                                     m_jointAccelerationIndex.offset,
                                     m_jointAccelerationIndex.size,
                                     m_jointAccelerationIndex.size)
            = -(iDynTree::toEigen(m_massMatrix)
                    .bottomRightCorner(m_jointAccelerationIndex.size, m_jointAccelerationIndex.size)
                + iDynTree::toEigen(m_reflectedInertia));

    // store the jacobians
    for (const auto& frame : m_framesInContact)
    {
        m_kinDynPtr->getFrameFreeFloatingJacobian(frame.indexInModel, m_jacobianMatrix);
        iDynTree::toEigen(m_A).block(0,
                                     frame.indexRangeInElement.offset,
                                     m_jointAccelerationIndex.size + m_baseAccelerationIndex.size,
                                     frame.indexRangeInElement.size)
            = iDynTree::toEigen(m_jacobianMatrix).transpose();
    }

    return m_A;
}

const iDynTree::VectorDynSize& FloatingBaseMultiBodyDynamicsElement::getB()
{
    m_kinDynPtr->generalizedBiasForces(m_generalizedBiasForces);
    iDynTree::toEigen(m_b).head(m_baseAccelerationIndex.size)
        = iDynTree::toEigen(m_generalizedBiasForces.baseWrench());
    iDynTree::toEigen(m_b).tail(m_jointAccelerationIndex.size)
        = iDynTree::toEigen(m_generalizedBiasForces.jointTorques());
    return m_b;
}
