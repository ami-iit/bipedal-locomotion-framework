/**
 * @file BaseVelocityTrackingTask.cpp
 * @authors Davide Gorbani
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/IK/BaseVelocityTrackingTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::IK;

bool BaseVelocityTrackingTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[BaseVelocityTrackingTask::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool BaseVelocityTrackingTask::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    if (!m_isInitialized)
    {
        log()->error("[BaseVelocityTrackingTask::setVariablesHandler] The task is not initialized. "
                     "Please call "
                     "initialize method.");
        return false;
    }

    if (!variablesHandler.getVariable(m_robotVelocityVariableName, m_robotVelocityVariable))
    {
        log()->error("[BaseVelocityTrackingTask::setVariablesHandler] Unable to get the variable "
                     "named {}.",
                     m_robotVelocityVariableName);
        return false;
    }

    if (m_robotVelocityVariable.size != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        log()->error("[BaseVelocityTrackingTask::setVariablesHandler] The size of the robot "
                     "velocity variable is "
                     "different from the one expected. Expected size: {}. Given size: {}.",
                     m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize,
                     m_robotVelocityVariable.size);
        return false;
    }

    // resize the matrices
    m_A.resize(m_DoFs, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_A.block(0, 0, m_DoFs, m_DoFs).setIdentity();
    m_b.resize(m_DoFs);
    m_b.setZero();

    return true;
}

bool BaseVelocityTrackingTask::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[BaseVelocityTrackingTask::initialize]";

    if (m_isInitialized)
    {
        log()->error("{} The task has already been initialized.", errorPrefix);
        return false;
    }

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

    if (!ptr->getParameter("robot_velocity_variable_name", m_robotVelocityVariableName))
    {
        log()->error("{} Error while retrieving the robot velocity variable.", errorPrefix);
        return false;
    }

    m_DoFs = m_spatialVelocitySize;

    m_desiredBaseVelocity = Eigen::VectorXd::Zero(m_spatialVelocitySize);

    // set the description
    m_description = "Base velocity regularization task";

    m_isInitialized = true;

    return true;
}

bool BaseVelocityTrackingTask::update()
{
    constexpr auto errorPrefix = "[BaseVelocityTrackingTask::update]";

    m_isValid = false;

    if (!m_isInitialized)
    {
        log()->error("{} The task is not initialized. Please call initialize method.", errorPrefix);
        return m_isValid;
    }

    m_b = m_desiredBaseVelocity;

    m_isValid = true;
    return m_isValid;
}

bool BaseVelocityTrackingTask::setSetPoint(Eigen::Ref<const Eigen::VectorXd> baseVelocity)
{
    constexpr auto errorPrefix = "[BaseVelocityTrackingTask::setSetPoint]";

    if (baseVelocity.size() != m_spatialVelocitySize)
    {
        log()->error("{} Wrong size of the desired reference setpoint. Expected size: {}. Given "
                     "size: {}.",
                     errorPrefix,
                     m_spatialVelocitySize,
                     baseVelocity.size());
        return false;
    }

    m_desiredBaseVelocity = baseVelocity;

    return true;
}

std::size_t BaseVelocityTrackingTask::size() const
{
    constexpr auto errorMessage = "[BaseVelocityTrackingTask::size] Please call setKinDyn "
                                  "method before. A size equal "
                                  "to zero will be returned.";

    assert((m_kinDyn != nullptr) && errorMessage);

    if (m_kinDyn == nullptr)
    {
        log()->error(errorMessage);
        return 0;
    }

    return m_DoFs;
}

BaseVelocityTrackingTask::Type BaseVelocityTrackingTask::type() const
{
    return Type::equality;
}

bool BaseVelocityTrackingTask::isValid() const
{
    return m_isValid;
}
