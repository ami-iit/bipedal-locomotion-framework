/**
 * @file GravityTask.cpp
 * @authors Ehsan Ranjbari
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/IK/GravityTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;

bool GravityTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[GravityTask::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool GravityTask::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    if (!m_isInitialized)
    {
        log()->error("[GravityTask::setVariablesHandler] The task is not initialized. Please call "
                     "initialize method.");
        return false;
    }

    // get the variable
    if (!variablesHandler.getVariable(m_robotVelocityVariable.name, m_robotVelocityVariable))
    {
        log()->error("[GravityTask::setVariablesHandler] Unable to get the variable named {}.",
                     m_robotVelocityVariable.name);
        return false;
    }

    // get the variable
    if (m_robotVelocityVariable.size != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        log()->error("[GravityTask::setVariablesHandler] The size of the robot velocity variable "
                     "is different from the one expected. Expected size: {}. Given size: {}.",
                     m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize,
                     m_robotVelocityVariable.size);
        return false;
    }

    // resize the matrices
    m_A.resize(m_DoFs, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_DoFs);
    m_b.setZero();
    m_jacobian.resize(m_spatialVelocitySize,
                      m_spatialVelocitySize + m_kinDyn->getNrOfDegreesOfFreedom());
    m_jacobian.setZero();
    m_desiredZDirectionBody.setZero();
    m_feedForwardBody.setZero();
    m_Am.resize(2, 3);
    m_bm.resize(2, 3);
    // clang-format off
    m_Am << 1, 0, 0,
            0, 1, 0;
    m_bm << 0, -1, 0,
            1, 0, 0;
    // clang-format on

    return true;
}

bool GravityTask::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[GravityTask::initialize]";

    m_description = "GravityTask";

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        log()->error("{} [{}] KinDynComputations object is not valid.", errorPrefix, m_description);

        return false;
    }

    if (m_kinDyn->getFrameVelocityRepresentation()
        != iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION)
    {
        log()->error("{} [{}] task supports only quantities expressed in MIXED "
                     "representation. Please provide a KinDynComputations with Frame velocity "
                     "representation set to MIXED_REPRESENTATION.",
                     errorPrefix,
                     m_description);
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} [{}] The parameter handler is not valid.", errorPrefix, m_description);
        return false;
    }

    std::string robotVelocityVariableName;
    if (!ptr->getParameter("robot_velocity_variable_name", m_robotVelocityVariable.name))
    {
        log()->error("{} [{}] while retrieving the robot velocity variable.",
                     errorPrefix,
                     m_description);
        return false;
    }

    // set the gains for the controllers
    if (!ptr->getParameter("kp", m_kp))
    {
        log()->error("{} [{}] to get the proportional linear gain.", errorPrefix, m_description);
        return false;
    }

    std::string targetFrameName;
    if (!ptr->getParameter("target_frame_name", targetFrameName))
    {
        log()->error("{} [{}] to get the end effector frame name.", errorPrefix, m_description);
        return false;
    }

    m_targetFrameIndex = m_kinDyn->getFrameIndex(targetFrameName);

    if (m_targetFrameIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} [{}] The specified target name ({}) does not seem to exist.",
                     errorPrefix,
                     m_description,
                     targetFrameName);
        return false;
    }

    m_isInitialized = true;

    return true;
}

bool GravityTask::update()
{
    using namespace iDynTree;

    m_isValid = false;

    auto targetRotation = toEigen(m_kinDyn->getWorldTransform(m_targetFrameIndex).getRotation());

    Eigen::Vector3d desiredZDirectionAbsolute = targetRotation * m_desiredZDirectionBody;
    Eigen::Vector3d feedforwardAbsolute = targetRotation * m_feedForwardBody;

    if (!m_kinDyn->getFrameFreeFloatingJacobian(m_targetFrameIndex, m_jacobian))
    {
        log()->error("[GravityTask::update] Unable to get the frame jacobian.");
        return m_isValid;
    }

    toEigen(this->subA(m_robotVelocityVariable)) = m_Am * m_jacobian.bottomRows<3>();
    m_b = feedforwardAbsolute.topRows<2>();
    m_b.noalias() -= m_kp * m_bm * desiredZDirectionAbsolute;

    m_isValid = true;
    return m_isValid;
}

bool GravityTask::setDesiredGravityDirectionInTargetFrame(
    const Eigen::Ref<const Eigen::Vector3d> desiredGravityDirection)
{
    m_desiredZDirectionBody = desiredGravityDirection;
    m_desiredZDirectionBody.normalize();
    return true;
}

bool GravityTask::setFeedForwardVelocityInTargetFrame(
    const Eigen::Ref<const Eigen::Vector3d> feedforwardVelocity)
{
    m_feedForwardBody = feedforwardVelocity;
    return true;
}

bool GravityTask::setSetPoint(const Eigen::Ref<const Eigen::Vector3d> desiredGravityDirection,
                              const Eigen::Ref<const Eigen::Vector3d> feedforwardVelocity)
{
    return setDesiredGravityDirectionInTargetFrame(desiredGravityDirection)
           && setFeedForwardVelocityInTargetFrame(feedforwardVelocity);
}

std::size_t GravityTask::size() const
{
    return m_DoFs;
}

GravityTask::Type GravityTask::type() const
{
    return Type::equality;
}

bool GravityTask::isValid() const
{
    return m_isValid;
}
