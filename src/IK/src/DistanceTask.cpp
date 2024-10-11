/**
 * @file DistanceTask.cpp
 * @authors Ehsan Ranjbari
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/IK/DistanceTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;

bool DistanceTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[DistanceTask::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool DistanceTask::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    if (!m_isInitialized)
    {
        log()->error("[DistanceTask::setVariablesHandler] The task is not initialized. Please call "
                     "initialize method.");
        return false;
    }

    // get the variable
    if (!variablesHandler.getVariable(m_robotVelocityVariable.name, m_robotVelocityVariable))
    {
        log()->error("[DistanceTask::setVariablesHandler] Unable to get the variable named {}.",
                     m_robotVelocityVariable.name);
        return false;
    }

    // get the variable
    if (m_robotVelocityVariable.size != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        log()->error("[DistanceTask::setVariablesHandler] The size of the robot velocity variable "
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
    m_relativeJacobian.resize(m_spatialVelocitySize, m_kinDyn->getNrOfDegreesOfFreedom());
    m_relativeJacobian.setZero();
    m_framePosition.setZero();

    return true;
}

bool DistanceTask::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[DistanceTask::initialize]";

    m_description = "DistanceTask";

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

    if (!ptr->getParameter("robot_velocity_variable_name", m_robotVelocityVariable.name))
    {
        log()->error("{} [{}] Failed to retrieve the robot velocity variable.",
                     errorPrefix,
                     m_description);
        return false;
    }

    // set the gains for the controllers
    if (!ptr->getParameter("kp", m_kp))
    {
        log()->error("{} [{}] Failed to get the proportional linear gain.",
                     errorPrefix,
                     m_description);
        return false;
    }

    // set the base frame name
    if (!ptr->getParameter("reference_frame_name", m_referenceName))
    {
        log()->debug("{} [{}] No base_name specified. Using default \"\"",
                     errorPrefix,
                     m_description);
    }

    // set the finger tip frame Name
    if (!ptr->getParameter("target_frame_name", m_targetFrameName))
    {
        log()->error("{} [{}] Failed to get the end effector frame name.",
                     errorPrefix,
                     m_description);
        return false;
    }

    // set the gains for the controllers
    if (!ptr->getParameter("distance_numeric_threshold", m_distanceNumericThreshold))
    {
        log()->debug("{} [{}] No distance_numeric_threshold specified. Using default {}",
                     errorPrefix,
                     m_description,
                     m_distanceNumericThreshold);
    }

    if (m_referenceName != "")
    {
        m_referenceFrameIndex = m_kinDyn->getFrameIndex(m_referenceName);

        if (m_referenceFrameIndex == iDynTree::FRAME_INVALID_INDEX)
        {
            log()->error("{} [{}] The specified base name ({}) does not seem to exist.",
                         errorPrefix,
                         m_description,
                         m_referenceName);
            return false;
        }
    }

    m_targetFrameIndex = m_kinDyn->getFrameIndex(m_targetFrameName);

    if (m_targetFrameIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} [{}] The specified target name ({}) does not seem to exist.",
                     errorPrefix,
                     m_description,
                     m_targetFrameName);
        return false;
    }

    m_isInitialized = true;

    return true;
}

bool DistanceTask::update()
{
    using namespace BipedalLocomotion::Conversions;
    using namespace iDynTree;

    m_isValid = false;

    if (m_referenceName == "")
    {
        m_framePosition = toEigen(m_kinDyn->getWorldTransform(m_targetFrameIndex).getPosition());

        if (!m_kinDyn->getFrameFreeFloatingJacobian(m_targetFrameIndex, m_jacobian))
        {
            log()->error("[DistanceTask::update] Unable to get the jacobian.");
            return m_isValid;
        }

    } else
    {
        m_framePosition = toEigen(
            m_kinDyn->getRelativeTransform(m_referenceFrameIndex, m_targetFrameIndex).getPosition());

        if (!m_kinDyn->getRelativeJacobian(m_referenceFrameIndex,
                                           m_targetFrameIndex,
                                           m_relativeJacobian))
        {
            log()->error("[DistanceTask::update] Unable to get the relative jacobian.");
            return m_isValid;
        }

        m_jacobian.rightCols(m_kinDyn->getNrOfDegreesOfFreedom()) = m_relativeJacobian;
    }

    m_computedDistance = m_framePosition.norm();

    toEigen(this->subA(m_robotVelocityVariable)).noalias()
        = (m_framePosition.transpose() * m_jacobian.topRows<3>())
          / (std::max(m_distanceNumericThreshold, m_computedDistance));
    m_b(0) = m_kp * (m_desiredDistance - m_computedDistance);

    m_isValid = true;
    return m_isValid;
}

bool DistanceTask::setDesiredDistance(double desiredDistance)
{
    m_desiredDistance = std::abs(desiredDistance);
    return true;
}

bool DistanceTask::setSetPoint(double desiredDistance)
{
    return setDesiredDistance(desiredDistance);
}

double DistanceTask::getDistance() const
{
    return m_computedDistance;
}

std::size_t DistanceTask::size() const
{
    return m_DoFs;
}

DistanceTask::Type DistanceTask::type() const
{
    return Type::equality;
}

bool DistanceTask::isValid() const
{
    return m_isValid;
}
