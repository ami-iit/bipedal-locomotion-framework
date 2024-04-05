/**
 * @file JointTrackingTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TSID/JointTrackingTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::TSID;
using namespace BipedalLocomotion;

bool JointTrackingTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[JointTrackingTask::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool JointTrackingTask::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    constexpr auto errorPrefix = "[JointTrackingTask::setVariablesHandler]";

    System::VariablesHandler::VariableDescription robotAccelerationVariable;

    if (!m_isInitialized)
    {
        log()->error("{} The task is not initialized. Please call initialize method.", errorPrefix);
        return false;
    }

    if (!variablesHandler.getVariable(m_robotAccelerationVariableName, robotAccelerationVariable))
    {
        log()->error("{} Error while retrieving the robot acceleration variable.", errorPrefix);
        return false;
    }

    if (robotAccelerationVariable.size != m_kinDyn->getNrOfDegreesOfFreedom() + 6)
    {
        log()->error("{} The size of the robot acceleration variable does not match with the one "
                     "stored in kinDynComputations object. Expected: {}. Given: {}",
                     errorPrefix,
                     m_kinDyn->getNrOfDegreesOfFreedom() + 6,
                     robotAccelerationVariable.size);
        return false;
    }

    // resize the matrices
    m_A.resize(m_kinDyn->getNrOfDegreesOfFreedom(), variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_kinDyn->getNrOfDegreesOfFreedom());

    // A is constant
    // here we assume that the first robot acceleration is stored as [base_acceleration;
    // joint_acceleration]
    iDynTree::toEigen(this->subA(robotAccelerationVariable))
        .rightCols(m_kinDyn->getNrOfDegreesOfFreedom())
        .setIdentity();

    return true;
}

bool JointTrackingTask::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[JointTrackingTask::initialize]";

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        log()->error("{} KinDynComputations object is not valid.", errorPrefix);
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("robot_acceleration_variable_name", m_robotAccelerationVariableName))
    {
        log()->error("{} Error while retrieving the robot acceleration variable.", errorPrefix);
        return false;
    }

    // set the gains for the controllers
    m_kp.resize(m_kinDyn->getNrOfDegreesOfFreedom());
    m_kd.resize(m_kinDyn->getNrOfDegreesOfFreedom());
    if (!ptr->getParameter("kp", m_kp))
    {
        log()->error("{} Error while retrieving the proportional gain.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("kd", m_kd))
    {
        log()->info("{} The default derivative gain will be set.", errorPrefix);
        m_kd = 2 * m_kp.cwiseSqrt();
    }

    if (m_kp.size() != m_kinDyn->getNrOfDegreesOfFreedom())
    {
        log()->error("{} The size of the kp gain does not match with the one stored in kinDynComputations object. "
                     "Expected: {}. Given: {}",
                     errorPrefix,
                     m_kinDyn->getNrOfDegreesOfFreedom(),
                     m_kp.size());
        return false;
    }

    if (m_kd.size() != m_kinDyn->getNrOfDegreesOfFreedom())
    {
        log()->error("{} The size of the kd gain does not match with the one stored in kinDynComputations object. "
                     "Expected: {}. Given: {}",
                     errorPrefix,
                     m_kinDyn->getNrOfDegreesOfFreedom(),
                     m_kd.size());
        return false;
    }

    // set the description
    m_description = "Joint tracking task";

    m_zero = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());
    m_desiredJointPosition = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());
    m_desiredJointVelocity = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());
    m_desiredJointAcceleration = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());
    m_jointPosition = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());
    m_jointVelocity = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());

    m_isInitialized = true;
    return true;
}

bool JointTrackingTask::update()
{
    constexpr std::string_view errorPrefix = "[JointTrackingTask::update] ";

    m_isValid = false;

    if (!m_kinDyn->getJointPos(m_jointPosition))
    {
        std::cerr << errorPrefix << " Unable to get the joint position" << std::endl;
        return false;
    }

    if (!m_kinDyn->getJointVel(m_jointVelocity))
    {
        std::cerr << errorPrefix << " Unable to get the joint velocity" << std::endl;
        return false;
    }

    m_b = m_desiredJointAcceleration;
    m_b.noalias() += m_kp.asDiagonal() * (m_desiredJointPosition - m_jointPosition);
    m_b.noalias() += m_kd.asDiagonal() * (m_desiredJointVelocity - m_jointVelocity);

    m_isValid = true;

    return true;
}

bool JointTrackingTask::setSetPoint(Eigen::Ref<const Eigen::VectorXd> jointPosition)
{
    return this->setSetPoint(jointPosition, m_zero, m_zero);
}

bool JointTrackingTask::setSetPoint(Eigen::Ref<const Eigen::VectorXd> jointPosition,
                                     Eigen::Ref<const Eigen::VectorXd> jointVelocity)
{
    return this->setSetPoint(jointPosition, jointVelocity, m_zero);
}

bool JointTrackingTask::setSetPoint(Eigen::Ref<const Eigen::VectorXd> jointPosition,
                                     Eigen::Ref<const Eigen::VectorXd> jointVelocity,
                                     Eigen::Ref<const Eigen::VectorXd> jointAcceleration)
{
    constexpr std::string_view errorPrefix = "[JointTrackingTask::setSetpoint] ";

    if (jointPosition.size() != m_kinDyn->getNrOfDegreesOfFreedom()
        || jointVelocity.size() != m_kinDyn->getNrOfDegreesOfFreedom()
        || jointAcceleration.size() != m_kinDyn->getNrOfDegreesOfFreedom())
    {
        std::cerr << errorPrefix << "Wrong size of the desired reference trajectory:" << std::endl
                  << "Expected size: " << m_kinDyn->getNrOfDegreesOfFreedom() << std::endl
                  << "Joint position size: " << jointPosition.size() << std::endl
                  << "Joint velocity size: " << jointVelocity.size() << std::endl
                  << "Joint acceleration size: " << jointAcceleration.size() << std::endl;
        return false;
    }

    m_desiredJointPosition = jointPosition;
    m_desiredJointVelocity = jointVelocity;
    m_desiredJointAcceleration = jointAcceleration;

    return true;
}

std::size_t JointTrackingTask::size() const
{
    constexpr auto errorMessage = "[JointTrackingTask::size] Please call setKinDyn method before. "
                                  "A size equal to zero will be returned.";

    assert((m_kinDyn != nullptr) && errorMessage);

    if (m_kinDyn == nullptr)
    {
        log()->warn(errorMessage);
        return 0;
    }
    return m_kinDyn->getNrOfDegreesOfFreedom();
}

JointTrackingTask::Type JointTrackingTask::type() const
{
    return Type::equality;
}

bool JointTrackingTask::isValid() const
{
    return m_isValid;
}
