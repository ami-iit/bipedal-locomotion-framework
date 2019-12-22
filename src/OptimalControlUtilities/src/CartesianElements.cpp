/**
 * @file CartesianElements.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <stdexcept>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/CartesianElements.h>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

CartesianElement::CartesianElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                   const VariableHandler& handler,
                                   const Type& type,
                                   const std::string& frameName,
                                   const AxisName& axisName /* = AxisName::X */)
    : ControlTask(kinDyn)
    , m_type(type)
    , m_isInContact(false)
{

    m_name = "CartesianElement (frame name: " + frameName + " Type: ";

    // get the index
    m_jointAccelerationIndex = handler.getVariable("joint_accelerations");
    m_baseAccelerationIndex = handler.getVariable("base_acceleration");

    if (!m_baseAccelerationIndex.isValid())
        throw std::runtime_error("[CartesianElement::CartesianElement] Undefined base_acceleration "
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

        if (type != Type::POSITION && type != Type::ONE_DIMENSION)
            throw std::runtime_error("[CartesianElement::CartesianElement] The CoM element can be "
                                     "only POSITION or ONE_DIMENSION type");
    }
    switch (type)
    {
    case Type::POSE:
        m_positionPD = std::make_unique<LinearPD<iDynTree::Vector3>>();
        m_orientationPD = std::make_unique<OrientationPD>();

        // in this case the element of the jacobian matrix starts from index zero and the number of
        // rows are 6
        m_typeIndex.offset = 0;
        m_typeIndex.size = 6;

        m_name += "Pose";

        break;

    case Type::POSITION:
        m_positionPD = std::make_unique<LinearPD<iDynTree::Vector3>>();

        // in this case the element of the jacobian matrix starts from index zero and the number of
        // rows are 3
        m_typeIndex.offset = 0;
        m_typeIndex.size = 3;

        m_name += "Position";

        break;

    case Type::ORIENTATION:
        m_orientationPD = std::make_unique<OrientationPD>();

        // in this case the element of the jacobian matrix starts from index 3 and the number of
        // rows are 3
        m_typeIndex.offset = 3;
        m_typeIndex.size = 3;

        m_name += "Orientation";

        break;

    case Type::ONE_DIMENSION:
        m_oneDegreePD = std::make_unique<LinearPD<double>>();

        // in this case the element of the jacobian matrix starts from index 3 and the number of
        // rows are 3
        if (axisName == AxisName::X)
            m_typeIndex.offset = 0;
        else if (axisName == AxisName::Y)
            m_typeIndex.offset = 1;
        else
            m_typeIndex.offset = 2;

        m_typeIndex.size = 1;

        m_name += "One Dimension";

        break;
    }

    m_name += ")";

    // resize the vector b and the matrix A
    m_A.resize(m_typeIndex.size, handler.getNumberOfVariables());
    m_b.resize(m_typeIndex.size);
    m_A.zero();
    m_b.zero();
}

void CartesianElement::setLinearPDGains(const iDynTree::Vector3& kp, const iDynTree::Vector3& kd)
{
    if (m_type != Type::POSITION && m_type != Type::POSE)
        throw std::runtime_error("[CartesianElement::setDesiredTrajectory] The type of the "
                                 "Cartesian element is neither POSITION nor POSE. This function "
                                 "will not set the gains you are expected.");

    m_positionPD->setGains(kp, kd);
    return;
}

void CartesianElement::setOrientationPDGains(const double& c0, const double& c1, const double& c2)
{
    if (m_type != Type::ORIENTATION && m_type != Type::POSE)
        throw std::runtime_error("[CartesianElement::setDesiredTrajectory] The type of the "
                                 "Cartesian element is neither ORIENTATION nor POSE. This function "
                                 "will not set the gains you are expected.");

    m_orientationPD->setGains(c0, c1, c2);
    return;
}

void CartesianElement::setOneDegreePDGains(const double& kp, const double& kd)
{
    if (m_type != Type::ONE_DIMENSION)
        throw std::runtime_error("[CartesianElement::setDesiredTrajectory] The type of the "
                                 "cartesian element is not ONE_DIMENSION.");

    m_oneDegreePD->setGains(kp, kd);
    return;
}

void CartesianElement::setDesiredTrajectory(const iDynTree::Vector3& acceleration,
                                            const iDynTree::Vector3& velocity,
                                            const iDynTree::Vector3& position)
{
    if (m_type != Type::POSE && m_type != Type::POSITION)
        throw std::runtime_error("[CartesianElement::setDesiredTrajectory] The type of the "
                                 "cartesian element is neither POSITION or POSE.");

    m_positionPD->setDesiredTrajectory(acceleration, velocity, position);
    return;
}

void CartesianElement::setDesiredTrajectory(const iDynTree::Vector3& acceleration,
                                            const iDynTree::Vector3& velocity,
                                            const iDynTree::Rotation& rotation)
{
    if (m_type != Type::ORIENTATION && m_type != Type::POSE)
        throw std::runtime_error("[CartesianElement::setDesiredTrajectory] The type of the "
                                 "cartesian element is neither ORIENTATION or POSE.");

    m_orientationPD->setDesiredTrajectory(acceleration, velocity, rotation);
    return;
}

void CartesianElement::setDesiredTrajectory(const iDynTree::SpatialAcc& acceleration,
                                            const iDynTree::Twist& velocity,
                                            const iDynTree::Transform& transform)
{
    if (m_type != Type::POSE)
        throw std::runtime_error("[CartesianElement::setDesiredTrajectory] The type of the "
                                 "cartesian element is not POSE.");

    m_positionPD->setDesiredTrajectory(acceleration.getLinearVec3(),
                                        velocity.getLinearVec3(),
                                        transform.getPosition());
    m_orientationPD->setDesiredTrajectory(acceleration.getAngularVec3(),
                                           velocity.getAngularVec3(),
                                           transform.getRotation());
    return;
}

void CartesianElement::setDesiredTrajectory(const double& acceleration,
                                            const double& velocity,
                                            const double& position)
{
    if (m_type != Type::ONE_DIMENSION)
        throw std::runtime_error("[CartesianElement::setDesiredTrajectory] The type of the "
                                 "cartesian element is not in the ONE_DIMENSION");

    m_oneDegreePD->setDesiredTrajectory(acceleration, velocity, position);
    return;
}

void CartesianElement::isInContact(bool isInContact)
{
    m_isInContact = isInContact;
}

const iDynTree::VectorDynSize& CartesianElement::getB()
{
    if (m_type == Type::POSE)
    {
        // The CoM cannot be a contact element -- see the constructor
        iDynTree::toEigen(m_b) = -iDynTree::toEigen(m_kinDynPtr->getFrameBiasAcc(m_frameIndex));

        if (!m_isInContact)
        {
            // the first three elements are positions
            m_positionPD->setFeedback(m_kinDynPtr->getFrameVel(m_frameIndex).getLinearVec3(),
                                       m_kinDynPtr->getWorldTransform(m_frameIndex).getPosition());
            iDynTree::toEigen(m_b).head<3>()
                += iDynTree::toEigen(m_positionPD->getControllerOutput());

            // the first three elements are orientation
            m_orientationPD->setFeedback(m_kinDynPtr->getFrameVel(m_frameIndex).getAngularVec3(),
                                          m_kinDynPtr->getWorldTransform(m_frameIndex).getRotation());
            iDynTree::toEigen(m_b).tail<3>()
                += iDynTree::toEigen(m_orientationPD->getControllerOutput());
        }

    }

    else if (m_type == Type::ORIENTATION)
    {
        if (!m_isInContact)
        {
            m_orientationPD->setFeedback(m_kinDynPtr->getFrameVel(m_frameIndex).getAngularVec3(),
                                          m_kinDynPtr->getWorldTransform(m_frameIndex).getRotation());

            iDynTree::toEigen(m_b)
                = iDynTree::toEigen(m_orientationPD->getControllerOutput())
                  - iDynTree::toEigen(m_kinDynPtr->getFrameBiasAcc(m_frameIndex)).tail<3>();
        } else
        {
            iDynTree::toEigen(m_b)
                = -iDynTree::toEigen(m_kinDynPtr->getFrameBiasAcc(m_frameIndex)).tail<3>();
        }
    }

    else if (m_type == Type::POSITION)
    {
        if (m_frameIndex != -1)
        {
            if (!m_isInContact)
            {
                m_positionPD->setFeedback(m_kinDynPtr->getFrameVel(m_frameIndex).getLinearVec3(),
                                           m_kinDynPtr->getWorldTransform(m_frameIndex).getPosition());

                iDynTree::toEigen(m_b)
                    = iDynTree::toEigen(m_positionPD->getControllerOutput())
                      - iDynTree::toEigen(m_kinDynPtr->getFrameBiasAcc(m_frameIndex)).head<3>();
            } else
            {
                iDynTree::toEigen(m_b)
                    = -iDynTree::toEigen(m_kinDynPtr->getFrameBiasAcc(m_frameIndex)).head<3>();
            }

        } else
        {
            // the CoM cannot be in contact or not
            m_positionPD->setFeedback(m_kinDynPtr->getCenterOfMassVelocity(),
                                       m_kinDynPtr->getCenterOfMassPosition());

            iDynTree::toEigen(m_b) = iDynTree::toEigen(m_positionPD->getControllerOutput())
                                     - iDynTree::toEigen(m_kinDynPtr->getCenterOfMassBiasAcc());
        }
    }

    else if (m_type == Type::ONE_DIMENSION)
    {
        if (m_frameIndex != -1)
            m_oneDegreePD->setFeedback(m_kinDynPtr->getFrameVel(m_frameIndex)
                                            .getLinearVec3()(m_typeIndex.offset),
                                        m_kinDynPtr->getWorldTransform(m_frameIndex)
                                            .getPosition()(m_typeIndex.offset));
        else
            m_oneDegreePD->setFeedback(m_kinDynPtr->getCenterOfMassVelocity()(m_typeIndex.offset),
                                        m_kinDynPtr->getCenterOfMassPosition()(m_typeIndex.offset));

        // in this case b is only a number
        if (!m_isInContact)
        {
            m_b(0) = m_oneDegreePD->getControllerOutput()
                     - m_kinDynPtr->getFrameBiasAcc(m_frameIndex)(m_typeIndex.offset);
        } else
            m_b(0) = -m_kinDynPtr->getFrameBiasAcc(m_frameIndex)(m_typeIndex.offset);
    }

    return m_b;
}

const iDynTree::MatrixDynSize& CartesianElement::getA()
{
    // If the frameIndex is different from -1 means that the frame you are trying to control is not
    // the CoM
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
