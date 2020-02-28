/**
 * @file CartesianElements.tpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <stdexcept>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/CartesianElements.h>

namespace BipedalLocomotionControllers
{
namespace OptimalControlUtilities
{

template <CartesianElementType type, CartesianElementAxisName axisName>
CartesianElement<type, axisName>::CartesianElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                                   const ControllerType& controller,
                                                   const VariableHandler& handler,
                                                   const std::string& frameName)
    : ControlTask(kinDyn)
    , m_isInContact(false)
    , m_controller(controller)
{

    m_name = "CartesianElement (frame name: " + frameName + " Type: ";

    // get the index
    m_jointAccelerationIndex = handler.getVariable("joint_accelerations");
    m_baseAccelerationIndex = handler.getVariable("base_acceleration");

    if (!m_baseAccelerationIndex.isValid())
        throw std::runtime_error("[CartesianElement::CartesianElement] Undefined "
                                 "base_acceleration "
                                 "variable");

    if (!m_jointAccelerationIndex.isValid())
        throw std::runtime_error("[CartesianElement::CartesianElement] Undefined "
                                 "joint_accelerations variable");

    if (frameName != "CoM")
    {
        m_frameIndex = m_kinDynPtr->model().getFrameIndex(frameName);

        m_jacobian.resize(6, m_jointAccelerationIndex.size + m_baseAccelerationIndex.size);

        if (m_frameIndex == iDynTree::FRAME_INVALID_INDEX)
            throw std::runtime_error("[CartesianElement::CartesianElement] Frame named " + frameName
                                     + " is not defined in the model");
    } else
    {
        m_frameIndex = -1;
        m_jacobian.resize(3, m_jointAccelerationIndex.size + m_baseAccelerationIndex.size);

        if (type != CartesianElementType::POSITION)
            throw std::runtime_error("[CartesianElement::CartesianElement] The CoM element can "
                                     "be "
                                     "only POSITION or ONE_DIMENSION type");
    }

    // in this case the element of the jacobian matrix starts from index zero and the number of
    // rows are 6
    if constexpr (type == CartesianElementType::POSE)
    {
        m_typeIndex.offset = 0;
        m_typeIndex.size = 6;

        m_name += "Pose";
    } else if constexpr (type == CartesianElementType::POSITION)
    {
        if constexpr (axisName == CartesianElementAxisName::ALL)
        {
            // in this case the element of the jacobian matrix starts from index zero and the
            // number of rows are 3
            m_typeIndex.offset = 0;
            m_typeIndex.size = 3;
        } else
        {
            m_typeIndex.offset
                = static_cast<std::underlying_type_t<CartesianElementAxisName>>(axisName);
            m_typeIndex.size = 1;
        }
        m_name += "Position";
    } else
    {
        // in this case the element of the jacobian matrix starts from index 3 and the number of
        // rows are 3
        m_typeIndex.offset = 3;
        m_typeIndex.size = 3;

        m_name += "Orientation";
    }

    m_name += ")";

    // resize the vector b and the matrix A
    m_A.resize(m_typeIndex.size, handler.getNumberOfVariables());
    m_b.resize(m_typeIndex.size);
    m_A.zero();
    m_b.zero();
}

template <CartesianElementType type, CartesianElementAxisName axisName>
template <typename T, typename U, typename W>
void CartesianElement<type, axisName>::setReference(const T& feedforward,
                                                    const U& referenceDerivative,
                                                    const W& reference)
{
    m_controller.setDesiredTrajectory(feedforward, referenceDerivative, reference);
    return;
}

template <CartesianElementType type, CartesianElementAxisName axisName>
void CartesianElement<type, axisName>::isInContact(bool isInContact)
{
    m_isInContact = isInContact;
}

template <CartesianElementType type, CartesianElementAxisName axisName>
const iDynTree::VectorDynSize& CartesianElement<type, axisName>::getB()
{
    if constexpr (type == CartesianElementType::POSE)
    {
        // The CoM cannot be a contact element -- see the constructor
        iDynTree::toEigen(m_b) = -iDynTree::toEigen(m_kinDynPtr->getFrameBiasAcc(m_frameIndex));

        if (!m_isInContact)
        {
            std::cerr << "sono cartesian " << std::endl;
            std::cerr << m_kinDynPtr->getFrameVel(m_frameIndex).toString() << std::endl;
            std::cerr << m_kinDynPtr->getWorldTransform(m_frameIndex).toString() << std::endl;

            m_controller.setFeedback(m_kinDynPtr->getFrameVel(m_frameIndex),
                                     m_kinDynPtr->getWorldTransform(m_frameIndex));

            iDynTree::toEigen(m_b) += iDynTree::toEigen(m_controller.getControllerOutput());
        }

    }

    else if constexpr (type == CartesianElementType::ORIENTATION)
    {
        if (!m_isInContact)
        {
            m_controller.setFeedback(m_kinDynPtr->getFrameVel(m_frameIndex).getAngularVec3(),
                                      m_kinDynPtr->getWorldTransform(m_frameIndex).getRotation());

            iDynTree::toEigen(m_b)
                = iDynTree::toEigen(m_controller.getControllerOutput())
                  - iDynTree::toEigen(m_kinDynPtr->getFrameBiasAcc(m_frameIndex)).template tail<3>();
        } else
        {
            iDynTree::toEigen(m_b)
                = -iDynTree::toEigen(m_kinDynPtr->getFrameBiasAcc(m_frameIndex)).template tail<3>();
        }
    }

    else if constexpr (type == CartesianElementType::POSITION
                       && axisName == CartesianElementAxisName::ALL)
    {
        if (m_frameIndex != -1)
        {
            if (!m_isInContact)
            {
                m_controller.setFeedback(m_kinDynPtr->getFrameVel(m_frameIndex).getLinearVec3(),
                                          m_kinDynPtr->getWorldTransform(m_frameIndex).getPosition());

                iDynTree::toEigen(m_b)
                    = iDynTree::toEigen(m_controller.getControllerOutput())
                      - iDynTree::toEigen(m_kinDynPtr->getFrameBiasAcc(m_frameIndex))
                            .template head<3>();
            } else
            {
                iDynTree::toEigen(m_b)
                    = -iDynTree::toEigen(m_kinDynPtr->getFrameBiasAcc(m_frameIndex))
                           .template head<3>();
            }

        } else
        {
            // the CoM cannot be in contact or not
            m_controller.setFeedback(m_kinDynPtr->getCenterOfMassVelocity(),
                                      m_kinDynPtr->getCenterOfMassPosition());

            iDynTree::toEigen(m_b) = iDynTree::toEigen(m_controller.getControllerOutput())
                                     - iDynTree::toEigen(m_kinDynPtr->getCenterOfMassBiasAcc());
        }
    } else
    {
        if (m_frameIndex != -1)
            m_controller.setFeedback(m_kinDynPtr->getFrameVel(m_frameIndex)
                                          .getLinearVec3()(m_typeIndex.offset),
                                      m_kinDynPtr->getWorldTransform(m_frameIndex)
                                          .getPosition()(m_typeIndex.offset));
        else
            m_controller.setFeedback(m_kinDynPtr->getCenterOfMassVelocity()(m_typeIndex.offset),
                                      m_kinDynPtr->getCenterOfMassPosition()(m_typeIndex.offset));

        // in this case b is only a number
        if (!m_isInContact)
        {
            m_b(0) = m_controller.getControllerOutput()
                     - m_kinDynPtr->getFrameBiasAcc(m_frameIndex)(m_typeIndex.offset);
        } else
            m_b(0) = -m_kinDynPtr->getFrameBiasAcc(m_frameIndex)(m_typeIndex.offset);
    }

    return m_b;
}

template <CartesianElementType type, CartesianElementAxisName axisName>
const iDynTree::MatrixDynSize& CartesianElement<type, axisName>::getA()
{
    // If the frameIndex is different from -1 means that the frame you are trying to control is
    // not the CoM
    if (m_frameIndex != -1)
        m_kinDynPtr->getFrameFreeFloatingJacobian(m_frameIndex, m_jacobian);
    else
        m_kinDynPtr->getCenterOfMassJacobian(m_jacobian);

    // copy the part related to the base
    iDynTree::toEigen(m_A).block(0,
                                 m_baseAccelerationIndex.offset,
                                 m_typeIndex.size,
                                 m_baseAccelerationIndex.size)
        = iDynTree::toEigen(m_jacobian)
              .block(m_typeIndex.offset, 0, m_typeIndex.size, m_baseAccelerationIndex.size);

    // copy the part related to the joint
    iDynTree::toEigen(m_A).block(0,
                                 m_jointAccelerationIndex.offset,
                                 m_typeIndex.size,
                                 m_jointAccelerationIndex.size)
        = iDynTree::toEigen(m_jacobian)
              .block(m_typeIndex.offset,
                     m_baseAccelerationIndex.size,
                     m_typeIndex.size,
                     m_jointAccelerationIndex.size);

    return m_A;
}
} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers
