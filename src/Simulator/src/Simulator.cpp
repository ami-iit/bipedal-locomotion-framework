/**
 * @file Simulator.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/Simulator/Simulator.h>
#include <iDynTree/Core/EigenHelpers.h>

using namespace BipedalLocomotionControllers::Simulator;

Simulator::Simulator(const iDynTree::Model& model)
{
    m_numberOfDoF = model.getNrOfDOFs();
    m_totalMass = model.getTotalMass();

    m_kinDyn.loadRobotModel(model);
    m_kinDyn.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

    // resize quantities
    m_jointPosition.resize(m_numberOfDoF);
    m_jointVelocity.resize(m_numberOfDoF);
    m_jointAcceleration.resize(m_numberOfDoF);
    m_generalizedJointTorques.resize(m_numberOfDoF + 6);

    m_massMatrix.resize(m_numberOfDoF + 6, m_numberOfDoF + 6);
    m_massMatrixInverse.resize(m_numberOfDoF + 6, m_numberOfDoF + 6);

    m_generalizedBiasForces.resize(model);

    m_leftContact.jacobian.resize(6, m_numberOfDoF + 6);
    m_rightContact.jacobian.resize(6, m_numberOfDoF + 6);

    m_gravity.zero();
    m_gravity(2) = -9.81;

    m_state = State::NotInitialized;
}

bool Simulator::setBaseFrame(const iDynTree::FrameIndex& frameBaseIndex, const std::string& name)
{
    if (!m_kinDyn.isValid())
    {
        std::cerr << "[setBaseFrame] Please set the Robot model before calling this method."
                  << std::endl;
        return false;
    }

    // get the link where the desired baseFrame is attached
    iDynTree::LinkIndex linkBaseIndex = m_kinDyn.model().getFrameLink(frameBaseIndex);

    // get the main frame of the link. In iDynTree the base frame has to be the main frame
    // of the link
    std::string baseFrameName = m_kinDyn.model().getLinkName(linkBaseIndex);

    m_baseFrames.insert(
        {name, {baseFrameName, m_kinDyn.getRelativeTransform(frameBaseIndex, linkBaseIndex)}});

    return true;
}

bool Simulator::reset(const iDynTree::VectorDynSize& initialJointValues,
                      const iDynTree::Transform& leftFootTransform,
                      const iDynTree::Transform& rightFootTransform)
{
    if (m_state != State::Initialized && m_state != State::Ready && m_state != State::Running)
    {
        std::cerr << "[Simulator::reset] The simulator cannot be reset. Please initialize it"
                  << std::endl;
        return false;
    }

    m_generalizedJointTorques.zero();
    m_jointVelocity.zero();
    m_jointAcceleration.zero();
    m_jointPosition = initialJointValues;
    m_baseTwist.zero();
    m_baseAcceleration.zero();


    // compute the base position
    // -mg + length * width * k (-z) = 0
    // z = -mg / (length * width * k);
    double gravity = 9.81;

    iDynTree::Position leftFootPosition = leftFootTransform.getPosition();
    leftFootPosition(2) = -m_totalMass * gravity / (2 * m_length * m_width * m_springCoeff);

    iDynTree::Transform leftFootTransformOnCarpet(leftFootTransform.getRotation(),
                                                  leftFootPosition);

    m_kinDyn.setFloatingBase(m_baseFrames["left_foot"].first);

    if (!m_kinDyn.setRobotState(leftFootTransformOnCarpet * m_baseFrames["left_foot"].second,
                                m_jointPosition,
                                m_baseTwist,
                                m_jointVelocity,
                                m_gravity))
    {
        std::cerr << "[Simulator::reset] Unable to set the kinDynObject" << std::endl;
        return false;
    }

    m_baseTransform = m_kinDyn.getWorldTransform(m_baseFrame);

    m_kinDyn.setFloatingBase(m_baseFrames["root"].first);
    if (!m_kinDyn.setRobotState(m_baseTransform * m_baseFrames["root"].second,
                                m_jointPosition,
                                m_baseTwist,
                                m_jointVelocity,
                                m_gravity))
    {
        std::cerr << "[Simulator::reset] Unable to set the kinDynObject" << std::endl;
        return false;
    }

    // reset the integrators
    m_jointPositionIntegrator->reset(m_jointPosition);
    m_jointVelocityIntegrator->reset(m_jointVelocity);

    m_baseLinearVelocityIntegrator->reset(m_baseTwist.getLinearVec3());
    m_baseAngularVelocityIntegrator->reset(m_baseTwist.getAngularVec3());

    m_basePositionIntegrator->reset(m_baseTransform.getPosition());
    m_baseRotationIntegrator->reset(m_baseTransform.getRotation());

    m_leftContact.frameNullForce = leftFootTransform;
    m_rightContact.frameNullForce = rightFootTransform;

    // compute contact wrench
    m_leftContact.model->setState(
        {{"twist", m_kinDyn.getFrameVel(m_leftContact.indexInTheModel)},
            {"frame_transform", m_kinDyn.getWorldTransform(m_leftContact.indexInTheModel)},
            {"null_force_transform", m_leftContact.frameNullForce}});

    m_rightContact.model->setState(
        {{"twist", m_kinDyn.getFrameVel(m_rightContact.indexInTheModel)},
            {"frame_transform", m_kinDyn.getWorldTransform(m_rightContact.indexInTheModel)},
            {"null_force_transform", m_rightContact.frameNullForce}});


    m_state = State::Ready;

    return true;
}

bool Simulator::advance(const double& seconds /*= 0 */)
{
    using iDynTree::toEigen;

    if (m_state != State::Ready && m_state != State::Running)
    {
        std::cerr << "[Simulator::advance] The simulator cannot advance. Please reset it."
                  << std::endl;
        return false;
    }

    if (seconds < 0)
    {
        std::cerr << "[Simulator::advance] The number of seconds has to be positive number"
                  << std::endl;
        return false;
    }

    size_t numberOfIterations = 1;
    if (seconds != 0)
        numberOfIterations = static_cast<size_t>(seconds / m_dT);

    iDynTree::VectorDynSize generalizedBiasForcesVector(m_numberOfDoF + 6);
    Eigen::Matrix3d A;
    iDynTree::Matrix3x3 rotationMatrixRateOfChange;
    double gain = 0.05;

    for (size_t i = 0; i < numberOfIterations; i++)
    {
        if (m_controlMode == ControlMode::Torque)
        {
            if (!m_kinDyn.getFreeFloatingMassMatrix(m_massMatrix))
            {
                std::cerr << "[Simulator::advance] Unable to get the mass matrix" << std::endl;
                return false;
            }
            if (!m_kinDyn.getFrameFreeFloatingJacobian(m_leftContact.indexInTheModel,
                                                       m_leftContact.jacobian))
            {
                std::cerr << "[Simulator::advance] Unable to get the left foot jacobian"
                          << std::endl;
                return false;
            }

            if (!m_kinDyn.getFrameFreeFloatingJacobian(m_rightContact.indexInTheModel,
                                                       m_rightContact.jacobian))
            {
                std::cerr << "[Simulator::advance] Unable to get the right foot jacobian"
                          << std::endl;
                return false;
            }

            m_leftContact.model->setState(
                {{"twist", m_kinDyn.getFrameVel(m_leftContact.indexInTheModel)},
                    {"frame_transform", m_kinDyn.getWorldTransform(m_leftContact.indexInTheModel)},
                    {"null_force_transform", m_leftContact.frameNullForce}});

            m_rightContact.model->setState(
                {{"twist", m_kinDyn.getFrameVel(m_rightContact.indexInTheModel)},
                    {"frame_transform", m_kinDyn.getWorldTransform(m_rightContact.indexInTheModel)},
                    {"null_force_transform", m_rightContact.frameNullForce}});

            const iDynTree::Wrench& leftWrench = m_leftContact.model->getContactWrench();
            const iDynTree::Wrench& rightWrench = m_rightContact.model->getContactWrench();

            if (!m_kinDyn.generalizedBiasForces(m_generalizedBiasForces))
            {
                std::cerr << "[Simulator::advance] Unable to get the bias forces" << std::endl;
                return false;
            }

            toEigen(generalizedBiasForcesVector).head(6)
                = toEigen(m_generalizedBiasForces.baseWrench());

            toEigen(generalizedBiasForcesVector).tail(m_numberOfDoF)
                = toEigen(m_generalizedBiasForces.jointTorques());

            Eigen::VectorXd robotAcceleration
                = toEigen(m_massMatrix).ldlt()
                      .solve(-toEigen(generalizedBiasForcesVector)
                             + toEigen(m_leftContact.jacobian).transpose() * toEigen(leftWrench)
                             + toEigen(m_rightContact.jacobian).transpose() * toEigen(rightWrench)
                             + toEigen(m_generalizedJointTorques));

            toEigen(m_jointAcceleration) = robotAcceleration.tail(m_numberOfDoF);
            toEigen(m_baseAcceleration.getLinearVec3()) = robotAcceleration.head(3);
            toEigen(m_baseAcceleration.getAngularVec3()) = robotAcceleration.segment(3, 3);
        }

        // integrate the position and the velocity. The order matters
        m_jointPosition = m_jointPositionIntegrator->integrate(m_jointVelocity);
        m_jointVelocity = m_jointVelocityIntegrator->integrate(m_jointAcceleration);

        iDynTree::Position basePosition;
        toEigen(basePosition) = toEigen(m_basePositionIntegrator->integrate(m_baseTwist.getLinearVec3()));
        m_baseTransform.setPosition(basePosition);

        A = gain * (toEigen((m_baseTransform.getRotation().inverse() * m_baseTransform.getRotation()).inverse())
               - Eigen::Matrix3d::Identity());

        toEigen(rotationMatrixRateOfChange) = (iDynTree::skew(toEigen(m_baseTwist.getAngularVec3())) + A) * toEigen(m_baseTransform.getRotation());

        Eigen::Matrix3d baseRotationEigen = toEigen(m_baseRotationIntegrator->integrate(rotationMatrixRateOfChange));
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(baseRotationEigen, Eigen::ComputeFullU | Eigen::ComputeFullV);

        iDynTree::Rotation baseRotation;
        iDynTree::toEigen(baseRotation) = svd.matrixU() * svd.matrixV().transpose();
        m_baseTransform.setRotation(baseRotation);

        m_baseTwist.getLinearVec3() = m_baseLinearVelocityIntegrator->integrate(m_baseAcceleration.getLinearVec3());
        m_baseTwist.getAngularVec3() = m_baseAngularVelocityIntegrator->integrate(m_baseAcceleration.getAngularVec3());

        // update kindyn
        if (!m_kinDyn.setRobotState(m_baseTransform * m_baseFrames["root"].second,
                                    m_jointPosition,
                                    m_baseTwist,
                                    m_jointVelocity,
                                    m_gravity))
        {
            std::cerr << "[Simulator::advance] Unable to set the kinDynObject at iteration number "
                      << i << std::endl;
            return false;
        }
    }

    m_leftContact.model->setState(
        {{"twist", m_kinDyn.getFrameVel(m_leftContact.indexInTheModel)},
            {"frame_transform", m_kinDyn.getWorldTransform(m_leftContact.indexInTheModel)},
            {"null_force_transform", m_leftContact.frameNullForce}});

    m_rightContact.model->setState(
        {{"twist", m_kinDyn.getFrameVel(m_rightContact.indexInTheModel)},
            {"frame_transform", m_kinDyn.getWorldTransform(m_rightContact.indexInTheModel)},
            {"null_force_transform", m_rightContact.frameNullForce}});

    return true;
}

bool Simulator::setAccelerationReferences(const iDynTree::VectorDynSize& acceleration)
{
    if (m_controlMode != ControlMode::Acceleration)
    {
        std::cerr << "[Simulator::setAccelerationReferences] The set control mode is not "
                     "acceleration."
                  << std::endl;
        return false;
    }

    if (acceleration.size() != m_numberOfDoF + 6)
    {
        std::cerr << "[Simulator::setAccelerationReferences] The number of desired acceleration is "
                     "different from the number of DoFs + 6. Number of DoF + 6"
                  << m_numberOfDoF + 6 << " size of desired torque vector " << acceleration.size()
                  << std::endl;
        return false;
    }

    iDynTree::toEigen(m_jointAcceleration) = iDynTree::toEigen(acceleration).tail(m_numberOfDoF);
    iDynTree::toEigen(m_baseAcceleration.getLinearVec3()) = iDynTree::toEigen(acceleration).head(3);
    iDynTree::toEigen(m_baseAcceleration.getAngularVec3()) = iDynTree::toEigen(acceleration).segment(3,3);

    return true;
}

bool Simulator::setTorqueReferences(const iDynTree::VectorDynSize& torques)
{
    if (m_controlMode != ControlMode::Torque)
    {
        std::cerr << "[Simulator::setTorqueReferences] The set control mode is not "
                     "torque."
                  << std::endl;
        return false;
    }

    if (torques.size() != m_numberOfDoF)
    {
        std::cerr << "[Simulator::setTorqueReferences] The number of desired torques is different "
                     "from the number of DoFs. Number of DoF "
                  << m_numberOfDoF << " size of desired torque vector " << torques.size()
                  << std::endl;
        return false;
    }

    iDynTree::toEigen(m_generalizedJointTorques).tail(m_numberOfDoF) = iDynTree::toEigen(torques);

    return true;
}

void Simulator::setLeftFootNullForceTransform(const iDynTree::Transform& transform)
{
    m_leftContact.frameNullForce = transform;

    m_leftContact.model->setState(
        {{"twist", m_kinDyn.getFrameVel(m_leftContact.indexInTheModel)},
            {"frame_transform", m_kinDyn.getWorldTransform(m_leftContact.indexInTheModel)},
            {"null_force_transform", m_leftContact.frameNullForce}});
}

void Simulator::setRightFootNullForceTransform(const iDynTree::Transform& transform)
{
    m_rightContact.frameNullForce = transform;

    m_rightContact.model->setState(
        {{"twist", m_kinDyn.getFrameVel(m_rightContact.indexInTheModel)},
            {"frame_transform", m_kinDyn.getWorldTransform(m_rightContact.indexInTheModel)},
            {"null_force_transform", m_rightContact.frameNullForce}});
}

iDynTree::Wrench Simulator::leftWrench()
{
    if(!m_leftContact.isInContact)
        return iDynTree::Wrench::Zero();

    return m_leftContact.model->getContactWrench();
}

iDynTree::Wrench Simulator::rightWrench()
{
    if(!m_rightContact.isInContact)
        return iDynTree::Wrench::Zero();

    return m_rightContact.model->getContactWrench();
}

void Simulator::setLeftFootState(bool isInContact)
{
    m_leftContact.isInContact = isInContact;
}

void Simulator::setRightFootState(bool isInContact)
{
    m_rightContact.isInContact = isInContact;
}

const iDynTree::VectorDynSize& Simulator::jointPositions() const
{
    return m_jointPosition;
}

const iDynTree::VectorDynSize& Simulator::jointVelocities() const
{
    return m_jointVelocity;
}

const iDynTree::Transform& Simulator::baseTransform() const
{
    return m_baseTransform;
}

const iDynTree::Twist& Simulator::baseVelocity() const
{
    return m_baseTwist;
}
