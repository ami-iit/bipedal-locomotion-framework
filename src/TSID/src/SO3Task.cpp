/**
 * @file SO3Task.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/TSID/SO3Task.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;

bool SO3Task::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    if (!m_isInitialized)
    {
        log()->error("[SO3Task::setVariablesHandler] The task is not initialized. Please call "
                     "initialize method.");
        return false;
    }

    // get the variable
    if (!variablesHandler.getVariable(m_robotAccelerationVariable.name,
                                      m_robotAccelerationVariable))
    {
        log()->error("[SO3Task::setVariablesHandler] Unable to get the variable named {}.",
                     m_robotAccelerationVariable.name);
        return false;
    }

    // get the variable
    if (m_robotAccelerationVariable.size
        != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        log()->error("[SO3Task::setVariablesHandler] The size of the robot velocity variable is "
                     "different from the one expected. Expected size: {}. Given size: {}.",
                     m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize,
                     m_robotAccelerationVariable.size);
        return false;
    }

    // resize the matrices
    m_A.resize(m_angularVelocitySize, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_angularVelocitySize);
    m_jacobian.resize(m_spatialVelocitySize,
                      m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize);

    return true;
}

bool SO3Task::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[SO3Task::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool SO3Task::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr std::string_view errorPrefix = "[SO3Task::initialize] ";

    std::string frameName = "Unknown";
    constexpr auto descriptionPrefix = "SO3Task Optimal Control Element - Frame name: ";

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        log()->error("{}, [{} {}] KinDynComputations object is not valid.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    if (m_kinDyn->getFrameVelocityRepresentation()
        != iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION)
    {
        log()->error("{}, [{} {}] The task supports only quantities expressed in MIXED "
                     "representation. Please provide a KinDynComputations with Frame velocity "
                     "representation set to MIXED_REPRESENTATION.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{}, [{} {}] The parameter handler is not valid.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    if (!ptr->getParameter("robot_acceleration_variable_name", m_robotAccelerationVariable.name))
    {
        log()->error("{}, [{} {}] Error while retrieving the robot velocity variable.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    if (!ptr->getParameter("frame_name", frameName)
        || (m_frameIndex = m_kinDyn->model().getFrameIndex(frameName))
               == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{}, [{} {}] Error while retrieving the frame that should be controlled.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    // set the gains for the SO3 controller
    Eigen::Vector3d kpAngular, kdAngular;
    double scalarBuffer;

    if (ptr->getParameter("kp_angular", scalarBuffer))
    {
        kpAngular.setConstant(scalarBuffer);
    }
    else if(!ptr->getParameter("kp_angular", kpAngular))
    {
        log()->error("{}, [{} {}] Unable to get the proportional angular gain.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    if (ptr->getParameter("kd_angular", scalarBuffer))
    {
        kdAngular.setConstant(scalarBuffer);
    }
    else if(!ptr->getParameter("kd_angular", kdAngular))
    {
        log()->error("{}, [{} {}] Unable to get the derivative angular gain.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    m_SO3Controller.setGains(kpAngular, kdAngular);

    // set the description
    m_description = std::string(descriptionPrefix) + frameName + ".";

    m_isInitialized = true;

    return true;
}

bool SO3Task::update()
{
    m_isValid = false;

    if (!m_isInitialized)
    {
        log()->error("[SO3Task::update] Please call initialize() before update().");
        return m_isValid;
    }

    m_SO3Controller.setState(BipedalLocomotion::Conversions::toManifRot(
                                 m_kinDyn->getWorldTransform(m_frameIndex).getRotation()),
                             iDynTree::toEigen(
                                 m_kinDyn->getFrameVel(m_frameIndex).getAngularVec3()));

    m_SO3Controller.computeControlLaw();


    m_b = -iDynTree::toEigen(m_kinDyn->getFrameBiasAcc(m_frameIndex)).tail<3>();
    m_b += m_SO3Controller.getControl().coeffs();

    if (!m_kinDyn->getFrameFreeFloatingJacobian(m_frameIndex, m_jacobian))
    {
        log()->error("[SO3Task::update] Unable to get the jacobian.");
        return m_isValid;
    }

    iDynTree::toEigen(this->subA(m_robotAccelerationVariable)) = m_jacobian.bottomRows<3>();

    m_isValid = true;
    return m_isValid;
}

bool SO3Task::setSetPoint(const manif::SO3d& I_R_F,
                          const manif::SO3d::Tangent& angularVelocity,
                          const manif::SO3d::Tangent& angularAcceleration)
{
    bool ok = true;

    ok = ok && m_SO3Controller.setDesiredState(I_R_F, angularVelocity);
    ok = ok && m_SO3Controller.setFeedForward(angularAcceleration);

    return ok;
}

std::size_t SO3Task::size() const
{
    return m_angularVelocitySize;
}

SO3Task::Type SO3Task::type() const
{
    return Type::equality;
}

bool SO3Task::isValid() const
{
    return m_isValid;
}
