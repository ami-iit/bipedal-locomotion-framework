/**
 * @file JointsTrackingTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/TSID/JointsTrackingTask.h>

#include <iDynTree/Core/EigenHelpers.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::TSID;

bool JointsTrackingTask::initialize(
    std::weak_ptr<ParametersHandler::IParametersHandler> paramHandler,
    const System::VariablesHandler& variablesHandler)
{
    constexpr std::string_view errorPrefix = "[JointsTrackingTask::initialize] ";

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        std::cerr << errorPrefix << "KinDynComputations object is not valid." << std::endl;
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        std::cerr << errorPrefix << "The parameter handler is not valid." << std::endl;
        return false;
    }

    System::VariablesHandler::VariableDescription robotAccelerationVariable;
    std::string robotAccelerationVariableName;
    if (!ptr->getParameter("robot_acceleration_variable_name", robotAccelerationVariableName)
        || !variablesHandler.getVariable(robotAccelerationVariableName, robotAccelerationVariable))
    {
        std::cerr << errorPrefix << "Error while retrieving the robot acceleration variable."
                  << std::endl;
        return false;
    }

    if (robotAccelerationVariable.size != m_kinDyn->getNrOfDegreesOfFreedom() + 6)
    {
        std::cerr << errorPrefix << "Error while retrieving the robot acceleration variable."
                  << std::endl;
        return false;
    }

    // set the gains for the controllers
    m_kp.resize(m_kinDyn->getNrOfDegreesOfFreedom());
    m_kd.resize(m_kinDyn->getNrOfDegreesOfFreedom());
    if (!ptr->getParameter("kp", m_kp))
    {
        std::cerr << errorPrefix << "Error while retrieving the proportional gain." << std::endl;
        return false;
    }

    if (!ptr->getParameter("kd", m_kd))
    {
        std::cout << errorPrefix << "Error while retrieving the derivative gain." << std::endl;
        std::cout << errorPrefix << "The default kd will be set." << std::endl;

        m_kd = 2 * m_kp.cwiseSqrt();
    }

    // set the description
    m_description = "Joint tracking task";

    // resize the matrices
    m_A.resize(m_kinDyn->getNrOfDegreesOfFreedom(), variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_kinDyn->getNrOfDegreesOfFreedom());

    m_zero = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());
    m_desiredJointPosition = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());
    m_desiredJointVelocity = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());
    m_desiredJointAcceleration = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());
    m_jointPosition = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());
    m_jointVelocity = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());

    // A is constant
    // here we assume that the first robot acceleration is stored as [base_acceleration;
    // joint_acceleration]
    iDynTree::toEigen(this->subA(robotAccelerationVariable))
        .rightCols(m_kinDyn->getNrOfDegreesOfFreedom())
        .setIdentity();

    return true;
}

bool JointsTrackingTask::update()
{
    constexpr std::string_view errorPrefix = "[JointsTrackingTask::update] ";

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

    m_b = m_desiredJointAcceleration
          + m_kp.asDiagonal() * (m_desiredJointPosition - m_jointPosition)
          + m_kd.asDiagonal() * (m_desiredJointVelocity - m_jointVelocity);

    return true;
}

bool JointsTrackingTask::setSetpoint(Eigen::Ref<const Eigen::VectorXd> jointPosition)
{
    return this->setSetpoint(jointPosition, m_zero, m_zero);
}

bool JointsTrackingTask::setSetpoint(Eigen::Ref<const Eigen::VectorXd> jointPosition,
                                     Eigen::Ref<const Eigen::VectorXd> jointVelocity)
{
    return this->setSetpoint(jointPosition, jointVelocity, m_zero);
}

bool JointsTrackingTask::setSetpoint(Eigen::Ref<const Eigen::VectorXd> jointPosition,
                                     Eigen::Ref<const Eigen::VectorXd> jointVelocity,
                                     Eigen::Ref<const Eigen::VectorXd> jointAcceleration)
{
    constexpr std::string_view errorPrefix = "[JointsTrackingTask::setSetpoint] ";

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
    m_desiredJointAcceleration = jointVelocity;

    return true;
}
