/**
 * @file ControlProblemElements.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <stdexcept>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

// Control Problem Element Function
ControlProblemElement::ControlProblemElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
    : m_kinDynPtr(kinDyn)
{
}

int ControlProblemElement::getSize() const
{
    return m_A.rows();
}

const iDynTree::MatrixDynSize& ControlProblemElement::getA()
{
    return m_A;
}

const std::string& ControlProblemElement::getName() const
{
    return m_name;
}

// CostFunctionOrEqualityConstraintElement
CostFunctionOrEqualityConstraintElement::CostFunctionOrEqualityConstraintElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
    : ControlProblemElement(kinDyn)
{
}

const iDynTree::VectorDynSize& CostFunctionOrEqualityConstraintElement::getB()
{
    return m_b;
}

// InequalityConstraintElement
InequalityConstraintElement::InequalityConstraintElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
    : ControlProblemElement(kinDyn)
{
}

// CartesianElement
CartesianElement::CartesianElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                   const VariableHandler& handler,
                                   const Type& type,
                                   const std::string& frameName,
                                   const AxisName& axisName /* = AxisName::X */)
    : CostFunctionOrEqualityConstraintElement(kinDyn)
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
                                     + "is not defined in the model");
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
        m_positionPID = std::make_unique<PositionPID>();
        m_orientationPID = std::make_unique<OrientationPID>();

        // in this case the element of the jacobian matrix starts from index zero and the number of
        // rows are 6
        m_typeIndex.offset = 0;
        m_typeIndex.size = 6;

        m_name += "Pose";

        break;

    case Type::POSITION:
        m_positionPID = std::make_unique<PositionPID>();

        // in this case the element of the jacobian matrix starts from index zero and the number of
        // rows are 3
        m_typeIndex.offset = 0;
        m_typeIndex.size = 3;

        m_name += "Position";

        break;

    case Type::ORIENTATION:
        m_orientationPID = std::make_unique<OrientationPID>();

        // in this case the element of the jacobian matrix starts from index 3 and the number of
        // rows are 3
        m_typeIndex.offset = 3;
        m_typeIndex.size = 3;

        m_name += "Orientation";

        break;

    case Type::ONE_DIMENSION:

        m_oneDegreePID = std::make_unique<OneDegreePID>();

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

void CartesianElement::setLinearPIDGains(const iDynTree::Vector3& kp, const iDynTree::Vector3& kd)
{
    if (m_type != Type::POSITION && m_type != Type::POSE)
        throw std::runtime_error("[CartesianElement::setDesiredTrajectory] The type of the "
                                 "Cartesian element is neither POSITION nor POSE. This function "
                                 "will not set the gains you are expected.");

    m_positionPID->setGains(kp, kd);
    return;
}

void CartesianElement::setOrientationPIDGains(const double& c0, const double& c1, const double& c2)
{
    if (m_type != Type::ORIENTATION && m_type != Type::POSE)
        throw std::runtime_error("[CartesianElement::setDesiredTrajectory] The type of the "
                                 "Cartesian element is neither ORIENTATION nor POSE. This function "
                                 "will not set the gains you are expected.");

    m_orientationPID->setGains(c0, c1, c2);
    return;
}

void CartesianElement::setOneDegreePIDGains(const double& kp, const double& kd)
{
    if (m_type != Type::ONE_DIMENSION)
        throw std::runtime_error("[CartesianElement::setDesiredTrajectory] The type of the "
                                 "cartesian element is not ONE_DIMENSION.");

    m_oneDegreePID->setGains(kp, kd);
    return;
}

void CartesianElement::setDesiredTrajectory(const iDynTree::Vector3& acceleration,
                                            const iDynTree::Vector3& velocity,
                                            const iDynTree::Vector3& position)
{
    if (m_type != Type::POSE && m_type != Type::POSITION)
        throw std::runtime_error("[CartesianElement::setDesiredTrajectory] The type of the "
                                 "cartesian element is neither POSITION or POSE.");

    m_positionPID->setDesiredTrajectory(acceleration, velocity, position);
    return;
}

void CartesianElement::setDesiredTrajectory(const iDynTree::Vector3& acceleration,
                                            const iDynTree::Vector3& velocity,
                                            const iDynTree::Rotation& rotation)
{
    if (m_type != Type::ORIENTATION && m_type != Type::POSE)
        throw std::runtime_error("[CartesianElement::setDesiredTrajectory] The type of the "
                                 "cartesian element is neither ORIENTATION or POSE.");

    m_orientationPID->setDesiredTrajectory(acceleration, velocity, rotation);
    return;
}

void CartesianElement::setDesiredTrajectory(const iDynTree::SpatialAcc& acceleration,
                                            const iDynTree::Twist& velocity,
                                            const iDynTree::Transform& transform)
{
    if (m_type != Type::POSE)
        throw std::runtime_error("[CartesianElement::setDesiredTrajectory] The type of the "
                                 "cartesian element is not POSE.");

    m_positionPID->setDesiredTrajectory(acceleration.getLinearVec3(),
                                        velocity.getLinearVec3(),
                                        transform.getPosition());
    m_orientationPID->setDesiredTrajectory(acceleration.getAngularVec3(),
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

    m_oneDegreePID->setDesiredTrajectory(acceleration, velocity, position);
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
            m_positionPID->setFeedback(m_kinDynPtr->getFrameVel(m_frameIndex).getLinearVec3(),
                                       m_kinDynPtr->getWorldTransform(m_frameIndex).getPosition());
            iDynTree::toEigen(m_b).head<3>()
                += iDynTree::toEigen(m_positionPID->getControllerOutput());

            // the first three elements are orientation
            m_orientationPID->setFeedback(m_kinDynPtr->getFrameVel(m_frameIndex).getAngularVec3(),
                                          m_kinDynPtr->getWorldTransform(m_frameIndex).getRotation());
            iDynTree::toEigen(m_b).tail<3>()
                += iDynTree::toEigen(m_orientationPID->getControllerOutput());
        }

    }

    else if (m_type == Type::ORIENTATION)
    {
        if (!m_isInContact)
        {
            m_orientationPID->setFeedback(m_kinDynPtr->getFrameVel(m_frameIndex).getAngularVec3(),
                                          m_kinDynPtr->getWorldTransform(m_frameIndex).getRotation());

            iDynTree::toEigen(m_b)
                = iDynTree::toEigen(m_orientationPID->getControllerOutput())
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
                m_positionPID->setFeedback(m_kinDynPtr->getFrameVel(m_frameIndex).getLinearVec3(),
                                           m_kinDynPtr->getWorldTransform(m_frameIndex).getPosition());

                iDynTree::toEigen(m_b)
                    = iDynTree::toEigen(m_positionPID->getControllerOutput())
                      - iDynTree::toEigen(m_kinDynPtr->getFrameBiasAcc(m_frameIndex)).head<3>();
            } else
            {
                iDynTree::toEigen(m_b)
                    = -iDynTree::toEigen(m_kinDynPtr->getFrameBiasAcc(m_frameIndex)).head<3>();
            }

        } else
        {
            // the CoM cannot be in contact or not
            m_positionPID->setFeedback(m_kinDynPtr->getCenterOfMassVelocity(),
                                       m_kinDynPtr->getCenterOfMassPosition());

            iDynTree::toEigen(m_b) = iDynTree::toEigen(m_positionPID->getControllerOutput())
                                     - iDynTree::toEigen(m_kinDynPtr->getCenterOfMassBiasAcc());
        }
    }

    else if (m_type == Type::ONE_DIMENSION)
    {
        if (m_frameIndex != -1)
            m_oneDegreePID->setFeedback(m_kinDynPtr->getFrameVel(m_frameIndex)
                                            .getLinearVec3()(m_typeIndex.offset),
                                        m_kinDynPtr->getWorldTransform(m_frameIndex)
                                            .getPosition()(m_typeIndex.offset));
        else
            m_oneDegreePID->setFeedback(m_kinDynPtr->getCenterOfMassVelocity()(m_typeIndex.offset),
                                        m_kinDynPtr->getCenterOfMassPosition()(m_typeIndex.offset));

        // in this case b is only a number
        if (!m_isInContact)
        {
            m_b(0) = m_oneDegreePID->getControllerOutput()
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

// System Dynamics
SystemDynamicsElement::SystemDynamicsElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                             const VariableHandler& handler,
                                             const std::vector<std::pair<std::string, std::string>>& framesInContact)
    : CostFunctionOrEqualityConstraintElement(kinDyn)
{
    // if this constructor is called the motor reflected inertia will not be used
    m_useReflectedInertia = false;

    m_jointAccelerationIndex = handler.getVariable("joint_accelerations");
    m_baseAccelerationIndex = handler.getVariable("base_acceleration");
    m_jointTorqueIndex = handler.getVariable("joint_torques");

    m_name = "System Dynamics Element";

    if (!m_baseAccelerationIndex.isValid())
        throw std::runtime_error("[SystemDynamicsElement::SystemDynamicsElement] Undefined "
                                 "base_acceleration variable");

    if (!m_jointAccelerationIndex.isValid())
        throw std::runtime_error("[SystemDynamicsElement::SystemDynamicsElement] Undefined "
                                 "joint_accelerations variable");

    if (!m_jointTorqueIndex.isValid())
        throw std::runtime_error("[SystemDynamicsElement::SystemDynamicsElement] Undefined "
                                 "joint_torques variable");

    for (const auto& frame : framesInContact)
    {
        Frame frameInContact;
        frameInContact.indexRangeInElement = handler.getVariable(frame.first);
        frameInContact.indexInModel = m_kinDynPtr->model().getFrameIndex(frame.second);

        if (!frameInContact.indexRangeInElement.isValid())
            throw std::runtime_error("[SystemDynamicsElement::SystemDynamicsElement] Undefined "
                                     "frame named "
                                     + frame.first + "in the variableHandler");

        if (frameInContact.indexInModel == iDynTree::FRAME_INVALID_INDEX)
            throw std::runtime_error("[SystemDynamicsElement::SystemDynamicsElement] Undefined "
                                     "frame named "
                                     + frame.second + "in the model");

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

SystemDynamicsElement::SystemDynamicsElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                             const VariableHandler& handler,
                                             const std::vector<std::pair<std::string, std::string>>& framesInContact,
                                             const iDynTree::VectorDynSize& gamma,
                                             const iDynTree::VectorDynSize& motorsInertia,
                                             const iDynTree::VectorDynSize& harmonicDriveInertia,
                                             const double& r,
                                             const double& R,
                                             const double& t)
    : SystemDynamicsElement(kinDyn, handler, framesInContact)
{
    // if this Constructor is called the reflected inertia will be used
    m_useReflectedInertia = true;
    m_name += " (with Reflected Inertia)";
    unsigned int actuatedDoFs = m_jointTorqueIndex.size;

    m_reflectedInertia.resize(actuatedDoFs, actuatedDoFs);

    // check the size of the vectors
    if (gamma.size() != actuatedDoFs)
        throw std::runtime_error("[SystemDynamicsElement::SystemDynamicsElement] The size of the "
                                 "vector gamma is not equal to the actuatedDoFs. Expected: "
                                 + std::to_string(actuatedDoFs)
                                 + "retrieved: " + std::to_string(gamma.size()));

    if (motorsInertia.size() != actuatedDoFs)
        throw std::runtime_error("[SystemDynamicsElement::SystemDynamicsElement] The size of the "
                                 "vector motorsInertia is not equal to the actuatedDoFs. Expected: "
                                 + std::to_string(actuatedDoFs)
                                 + "retrieved: " + std::to_string(motorsInertia.size()));

    if (harmonicDriveInertia.size() != actuatedDoFs)
        throw std::runtime_error("[SystemDynamicsElement::SystemDynamicsElement] The size of the "
                                 "vector harmonicDriveInertia is not equal to the actuatedDoFs. "
                                 "Expected: "
                                 + std::to_string(actuatedDoFs)
                                 + "retrieved: " + std::to_string(harmonicDriveInertia.size()));

    iDynTree::VectorDynSize augmentedInertia(actuatedDoFs);

    iDynTree::toEigen(augmentedInertia)
        = iDynTree::toEigen(harmonicDriveInertia) + iDynTree::toEigen(motorsInertia);

    iDynTree::MatrixDynSize couplingMatrix(actuatedDoFs, actuatedDoFs);
    iDynTree::toEigen(couplingMatrix).setIdentity();

    // TODO do in a better way (HARD CODED JOINTS)
    iDynTree::toEigen(couplingMatrix).block(0, 0, 3, 3) << 0.5, -0.5, 0, 0.5, 0.5, 0, r / (2 * R),
        r / (2 * R), r / R;

    iDynTree::toEigen(couplingMatrix).block(3, 3, 3, 3) << -1, 0, 0, -1, -t, 0, 0, t, -t;

    iDynTree::toEigen(couplingMatrix).block(7, 7, 3, 3) << 1, 0, 0, 1, t, 0, 0, -t, t;

    iDynTree::toEigen(m_reflectedInertia)
        = (iDynTree::toEigen(couplingMatrix) * iDynTree::toEigen(gamma).asDiagonal())
              .inverse()
              .transpose()
          * iDynTree::toEigen(motorsInertia).asDiagonal()
          * (iDynTree::toEigen(couplingMatrix) * iDynTree::toEigen(gamma).asDiagonal()).inverse();
}

SystemDynamicsElement::SystemDynamicsElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                             const VariableHandler& handler,
                                             const std::vector<std::pair<std::string, std::string>>& framesInContact,
                                             const iDynTree::MatrixDynSize& regularizationMatrix)
    : SystemDynamicsElement(kinDyn, handler, framesInContact)
{
    m_useReflectedInertia = true;
    m_name += " (with regularization matrix)";

    unsigned int actuatedDoFs = m_jointTorqueIndex.size;
    if (regularizationMatrix.rows() != actuatedDoFs)
        throw std::runtime_error("[SystemDynamicsElement::SystemDynamicsElement] The number of "
                                 "rows of the regularizationMatrix is not equal to the one "
                                 "expected.  Expected: "
                                 + std::to_string(actuatedDoFs)
                                 + "retrieved: " + std::to_string(regularizationMatrix.rows()));

    if (regularizationMatrix.cols() != actuatedDoFs)
        throw std::runtime_error("[SystemDynamicsElement::SystemDynamicsElement] The number of "
                                 "columns of the regularizationMatrix is not equal to the one "
                                 "expected.  Expected: "
                                 + std::to_string(actuatedDoFs)
                                 + "retrieved: " + std::to_string(regularizationMatrix.cols()));

    m_reflectedInertia = regularizationMatrix;
}

const iDynTree::MatrixDynSize& SystemDynamicsElement::getA()
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

const iDynTree::VectorDynSize& SystemDynamicsElement::getB()
{
    m_kinDynPtr->generalizedBiasForces(m_generalizedBiasForces);
    iDynTree::toEigen(m_b).head(m_baseAccelerationIndex.size)
        = iDynTree::toEigen(m_generalizedBiasForces.baseWrench());
    iDynTree::toEigen(m_b).tail(m_jointAccelerationIndex.size)
        = iDynTree::toEigen(m_generalizedBiasForces.jointTorques());
    return m_b;
}

// Centroidal Linear Momentum
CentroidalLinearMomentumElement::CentroidalLinearMomentumElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const std::vector<std::string>& framesInContact)
    : CostFunctionOrEqualityConstraintElement(kinDyn)
{
    m_name = "Centroidal Linear Momentum Element";

    m_robotMass = m_kinDynPtr->model().getTotalMass();

    // resize and reset matrices
    m_A.resize(3, handler.getNumberOfVariables());
    m_A.zero();
    m_b.resize(3);
    m_b.zero();

    for (const std::string& frameName : framesInContact)
    {
        iDynTree::IndexRange variableIndex = handler.getVariable(frameName);
        if (!variableIndex.isValid())
            throw std::runtime_error("[CentroidalLinearMomentumElement::"
                                     "CentroidalLinearMomentumElement] Undefined frame named "
                                     + frameName + "in the variableHandler");

        iDynTree::toEigen(m_A).block(0, variableIndex.offset, 3, 3).setIdentity();
    }
}

void CentroidalLinearMomentumElement::setVRP(const iDynTree::Vector3& VRP)
{
    m_VRP = VRP;
}

const iDynTree::VectorDynSize& CentroidalLinearMomentumElement::getB()
{
    iDynTree::Position com;
    com = m_kinDynPtr->getCenterOfMassPosition();

    double gravity = 9.81;
    double omegaSquare = gravity / com(2);
    iDynTree::toEigen(m_b)
        = omegaSquare * m_robotMass * (iDynTree::toEigen(com) - iDynTree::toEigen(m_VRP));
    m_b(2) += gravity * m_robotMass;

    return m_b;
}

// Centroidal Angular momentum
CentroidalAngularMomentumElement::CentroidalAngularMomentumElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                                                   const VariableHandler& handler,
                                                                   const std::vector<std::pair<std::string, std::string>>& framesInContact)
    : CostFunctionOrEqualityConstraintElement(kinDyn)
{
    m_name = "Centroidal Angular Momentum Element";

    // resize and reset matrices
    m_A.resize(3, handler.getNumberOfVariables());
    m_A.zero();
    m_b.resize(3);
    m_b.zero();

    // initialize the PID
    m_pid = std::make_unique<PositionPID>();
    m_zero.zero();

    for (const auto& frame : framesInContact)
    {
        Frame frameInContact;
        frameInContact.indexRangeInElement = handler.getVariable(frame.first);
        frameInContact.indexInModel = m_kinDynPtr->model().getFrameIndex(frame.second);

        if (!frameInContact.indexRangeInElement.isValid())
            throw std::runtime_error("[CentroidalAngularMomentumElement::"
                                     "CentroidalAngularMomentumElement] Undefined frame named "
                                     + frame.first + "in the variableHandler");

        if (frameInContact.indexInModel == iDynTree::FRAME_INVALID_INDEX)
            throw std::runtime_error("[CentroidalAngularMomentumElement::"
                                     "CentroidalAngularMomentumElement] Undefined frame named "
                                     + frame.second + "in the model");

        m_framesInContact.push_back(frameInContact);

        // the matrix A relative to one contact is
        // A = [skew_symmetric; I]
        iDynTree::toEigen(m_A)
            .block(0, frameInContact.indexRangeInElement.offset + 3, 3, 3)
            .setIdentity();
    }
}

void CentroidalAngularMomentumElement::setGain(const double& kp)
{
    m_pid->setGains(kp, 0.0);
}

void CentroidalAngularMomentumElement::setDesiredCentroidalAngularMomentum(
    const iDynTree::Vector3& centroidalAngularMomentumVelocity,
    const iDynTree::Vector3& centroidalAngularMomentum)
{
    // TODO probably it can be optimized
    // u = centroidalAngularMomentumVelocity_des + kp (centroidalAngularMomentum_des -
    // centroidalAngularMomentum)
    m_pid->setDesiredTrajectory(centroidalAngularMomentumVelocity,
                                m_zero,
                                centroidalAngularMomentum);
}

const iDynTree::MatrixDynSize& CentroidalAngularMomentumElement::getA()
{
    iDynTree::Position com;
    com = m_kinDynPtr->getCenterOfMassPosition();

    for (const auto& frame : m_framesInContact)
    {
        iDynTree::toEigen(m_A).block(0, frame.indexRangeInElement.offset, 3, 3) = iDynTree::skew(
            iDynTree::toEigen(m_kinDynPtr->getWorldTransform(frame.indexInModel).getPosition())
            - iDynTree::toEigen(com));
    }
    return m_A;
}

const iDynTree::VectorDynSize& CentroidalAngularMomentumElement::getB()
{
    m_pid->setFeedback(m_zero, m_kinDynPtr->getCentroidalTotalMomentum().getAngularVec3());
    iDynTree::toEigen(m_b) = iDynTree::toEigen(m_pid->getControllerOutput());
    return m_b;
}

// RegularizationElement
RegularizationElement::RegularizationElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                             const VariableHandler& handler,
                                             const std::string& variableName)
    : CostFunctionOrEqualityConstraintElement(kinDyn)
{
    m_name = "Regularization element (variable: " + variableName + ")";

    iDynTree::IndexRange variableIndex = handler.getVariable(variableName);

    if (!variableIndex.isValid())
        throw std::runtime_error("[RegularizationElement::RegularizationElement] Undefined "
                                 "variable named "
                                 + variableName + "in the variableHandler");

    // resize and set the matrices
    m_A.resize(variableIndex.size, handler.getNumberOfVariables());
    m_A.zero();
    m_b.resize(variableIndex.size);
    m_b.zero();

    iDynTree::toEigen(m_A)
        .block(0, variableIndex.offset, variableIndex.size, variableIndex.size)
        .setIdentity();
}

// RegularizationWithControlElement
RegularizationWithControlElement::RegularizationWithControlElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const std::string& variableName)
    : RegularizationElement(kinDyn, handler, variableName)
{
    m_name = "Regularization with control element (variable: " + variableName + ")";

    iDynTree::IndexRange variableIndex = handler.getVariable(variableName);

    if (!variableIndex.isValid())
        throw std::runtime_error("[RegularizationWithControlElement::"
                                 "RegularizationWithControlElement] Undefined variable named "
                                 + variableName + "in the variableHandler");

    // resize quantities
    m_kp.resize(variableIndex.size);
    m_kd.resize(variableIndex.size);

    m_desiredPosition.resize(variableIndex.size);
    m_desiredVelocity.resize(variableIndex.size);
    m_desiredAcceleration.resize(variableIndex.size);

    m_position.resize(variableIndex.size);
    m_velocity.resize(variableIndex.size);
}

void RegularizationWithControlElement::setDesiredTrajectory(const iDynTree::VectorDynSize& acceleration,
                                                            const iDynTree::VectorDynSize& velocity,
                                                            const iDynTree::VectorDynSize& position)
{
    m_desiredAcceleration = acceleration;
    m_desiredVelocity = velocity;
    m_desiredPosition = position;
}

void RegularizationWithControlElement::setState(const iDynTree::VectorDynSize& velocity,
                                                const iDynTree::VectorDynSize& position)
{
    m_velocity = velocity;
    m_position = position;
}

void RegularizationWithControlElement::setPIDGains(const iDynTree::VectorDynSize& kp,
                                                   const iDynTree::VectorDynSize& kd)
{
    m_kp = kp;
    m_kd = kd;
}

const iDynTree::VectorDynSize& RegularizationWithControlElement::getB()
{
    iDynTree::toEigen(m_b)
        = iDynTree::toEigen(m_desiredAcceleration)
          + iDynTree::toEigen(m_kd).asDiagonal()
                * (iDynTree::toEigen(m_desiredVelocity) - iDynTree::toEigen(m_velocity))
          + iDynTree::toEigen(m_kp).asDiagonal()
                * (iDynTree::toEigen(m_desiredPosition) - iDynTree::toEigen(m_position));

    return m_b;
}

// ZMP Element
ZMPElement::ZMPElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                       const VariableHandler& handler,
                       const std::vector<std::pair<std::string, std::string>>& framesInContact)
    : CostFunctionOrEqualityConstraintElement(kinDyn)
{
    m_name = "ZMP element: (Frames in contact: ";

    // resize and reset matrices
    m_A.resize(2, handler.getNumberOfVariables());
    m_A.zero();
    m_b.resize(2);
    m_b.zero();

    m_contactFramePosition.zero();

    for (const auto& frameInContact : framesInContact)
    {
        Frame frame;
        frame.indexRangeInElement = handler.getVariable(frameInContact.first);
        frame.indexInModel = m_kinDynPtr->model().getFrameIndex(frameInContact.second);

        m_framesInContact.push_back(frame);

        if (!frame.indexRangeInElement.isValid())
            throw std::runtime_error("[ZMPElement::ZMPElement] Undefined frame named "
                                     + frameInContact.first + "in the variableHandler");

        if (frame.indexInModel == iDynTree::FRAME_INVALID_INDEX)
            throw std::runtime_error("[ZMPElement::ZMPElement] Undefined frame named "
                                     + frameInContact.second + "in the model");

        // constant values
        m_A(0, frame.indexRangeInElement.offset + 4) = 1;
        m_A(1, frame.indexRangeInElement.offset + 5) = -1;

        m_name += "[" + frameInContact.first + ",  " + frameInContact.second + "] ";
    }

    m_name += ")";
}

void ZMPElement::setDesiredZMP(const iDynTree::Vector2& ZMP)
{
    m_ZMP = ZMP;
}

const iDynTree::MatrixDynSize& ZMPElement::getA()
{
    for (const auto& frame : m_framesInContact)
    {
        m_contactFramePosition = m_kinDynPtr->getWorldTransform(frame.indexInModel).getPosition();

        m_A(0, frame.indexRangeInElement.offset + 2) = m_ZMP(0) - m_contactFramePosition(0);
        m_A(1, frame.indexRangeInElement.offset + 2) = m_ZMP(1) - m_contactFramePosition(1);
    }
    return m_A;
}

// ContactWrenchFeasibilityElement

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
                                 + frameInContact.first + "in the variableHandler");

    if (m_frameInContact.indexInModel == iDynTree::FRAME_INVALID_INDEX)
        throw std::runtime_error("[ContactWrenchFeasibilityElement::"
                                 "ContactWrenchFeasibilityElement] Undefined frame named "
                                 + frameInContact.second + "in the model");

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
        throw std::runtime_error("[SystemDynamicsElement::SystemDynamicsElement] Undefined "
                                 "variable named joint_accelerations in the variableHandler");

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
