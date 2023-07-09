/**
 * @file GravityTask.cpp
 * @authors Ehsan Ranjbari
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/IK/GravityTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

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
        log()->error("[GravityTask::setVariablesHandler] The size of the robot velocity variable is "
                     "different from the one expected. Expected size: {}. Given size: {}.",
                     m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize,
                     m_robotVelocityVariable.size);
        return false;
    }

    // resize the matrices
    m_A.resize(m_DoFs, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_DoFs);
    m_b.setZero();
    m_Angularjacobian.resize(3, variablesHandler.getNumberOfVariables());
    m_Angularjacobian.setZero();
    m_relativeJacobian.resize(6, m_kinDyn->getNrOfDegreesOfFreedom());
    m_relativeJacobian.setZero();
    m_currentAcc.setZero(3);
    m_currentGyro.setZero(3);
    m_currentAccNorm.setZero(3);
    m_Am.resize(2,3);
    m_Am << 1, 0, 0,
            0, 1, 0;
    m_bm.resize(2,3);
    m_bm << 0 ,1, 0,
            -1, 0, 0;

    return true;
}

bool GravityTask::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
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

    // set the base frame name
    if (!ptr->getParameter("base_name", m_baseName))
    {
        log()->debug("{} [{}] to get the base name. Using default \"\"",
                     errorPrefix,
                     m_description);
    }

    if (m_baseName != "")
    {
        m_baseIndex = m_kinDyn->getFrameIndex(m_baseName);

        if (m_baseIndex == iDynTree::FRAME_INVALID_INDEX)
            return false;
    }

    // set the finger tip frame Name
    if (!ptr->getParameter("target_frame_name", m_targetFrameName))
    {
        log()->error("{} [{}] to get the end effector frame name.", errorPrefix, m_description);
        return false;
    }

    m_targetFrameIndex = m_kinDyn->getFrameIndex(m_targetFrameName);

    if (m_targetFrameIndex == iDynTree::FRAME_INVALID_INDEX)
        return false;

    m_isInitialized = true;

    return true;
}

bool GravityTask::update()
{
    using namespace iDynTree;

    m_isValid = false;

    // Normalize Accelerometer vector
    m_accDenomNorm = sqrt(pow(m_currentAcc(0),2) + pow(m_currentAcc(1),2) + pow(m_currentAcc(2),2));

    for (size_t i = 0; i < m_currentAcc.size(); i++)
    {
        m_currentAccNorm(i) = m_currentAcc(i) / m_accDenomNorm;
    }

    m_currentAccNorm = toEigen(m_kinDyn->getWorldTransform(m_targetFrameName).getRotation()) * m_currentAccNorm;

    // get the relative jacobian
    if (!m_kinDyn->getRelativeJacobian(m_baseIndex, m_targetFrameIndex, m_relativeJacobian))
    {
        log()->error("[GravityTask::update] Unable to get the relative jacobian.");
        return m_isValid;
    }

    // get the angular part of the jacobian
    m_Angularjacobian.bottomRightCorner(3, m_kinDyn->getNrOfDegreesOfFreedom()) = m_relativeJacobian.bottomRightCorner(3, m_kinDyn->getNrOfDegreesOfFreedom());

    m_A = m_Am * m_Angularjacobian;
    m_b << ( m_kp * m_bm * m_currentAccNorm) + 1000 * m_currentGyro.block<2,1>(1,0);

    // A and b are now valid
    m_isValid = true;
    return m_isValid;
}

bool GravityTask::setEstimateGravityDir(const Eigen::Ref<const Eigen::VectorXd> currentGravityDir)
{
    m_currentAcc = currentGravityDir;

    return true;
}

bool GravityTask::setGyroscope(const Eigen::Ref<const Eigen::VectorXd> currentGyro)
{
    m_currentGyro = currentGyro * (M_PI)/180;

    return true;
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
