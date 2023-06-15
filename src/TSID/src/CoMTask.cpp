/**
 * @file CoMTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/TSID/CoMTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;

bool CoMTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[CoMTask::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool CoMTask::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    if (!m_isInitialized)
    {
        log()->error("[CoMTask::setVariablesHandler] The task is not initialized. Please call "
                     "initialize method.");
        return false;
    }

    // get the variable
    if (!variablesHandler.getVariable(m_robotAccelerationVariable.name,
                                      m_robotAccelerationVariable))
    {
        log()->error("[CoMTask::setVariablesHandler] Unable to get the variable named {}.",
                     m_robotAccelerationVariable.name);
        return false;
    }

    // get the variable
    if (m_robotAccelerationVariable.size
        != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        log()->error("[CoMTask::setVariablesHandler] The size of the robot velocity variable is "
                     "different from the one expected. Expected size: {}. Given size: {}.",
                     m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize,
                     m_robotAccelerationVariable.size);
        return false;
    }

    // resize the matrices
    m_A.resize(m_linearVelocitySize, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_linearVelocitySize);
    m_b.setZero();

    return true;
}

bool CoMTask::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[CoMTask::initialize]";

    m_description = "CoMTask Task.";

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

    if (!ptr->getParameter("robot_acceleration_variable_name", m_robotAccelerationVariable.name))
    {
        log()->error("{} [{}] while retrieving the robot velocity variable.",
                     errorPrefix,
                     m_description);
        return false;
    }

    double scalarBuffer;
    Eigen::Vector3d kpLinear, kdLinear;
    if (ptr->getParameter("kp_linear", scalarBuffer))
    {
        kpLinear.setConstant(scalarBuffer);
    } else if (!ptr->getParameter("kp_linear", kpLinear))
    {
        log()->error("{}, [{}] Unable to get the proportional gain.", errorPrefix, m_description);
        return false;
    }

    if (ptr->getParameter("kd_linear", scalarBuffer))
    {
        kdLinear.setConstant(scalarBuffer);
    } else if (!ptr->getParameter("kd_linear", kdLinear))
    {
        log()->error("{}, [{}] Unable to get the derivative gain.", errorPrefix, m_description);
        return false;
    }

    m_R3Controller.setGains(std::move(kpLinear), std::move(kdLinear));
    m_isInitialized = true;

    return true;
}

bool CoMTask::update()
{
    using namespace BipedalLocomotion::Conversions;
    using namespace iDynTree;

    m_isValid = false;

    // set the state
    m_R3Controller.setState(toEigen(m_kinDyn->getCenterOfMassPosition()),
                            toEigen(m_kinDyn->getCenterOfMassVelocity()));

    // update the controller
    m_R3Controller.computeControlLaw();

    m_b = m_R3Controller.getControl().coeffs()
          - iDynTree::toEigen(m_kinDyn->getCenterOfMassBiasAcc());

    // get the CoM jacobian
    if (!m_kinDyn->getCenterOfMassJacobian(this->subA(m_robotAccelerationVariable)))
    {
        log()->error("[CoMTask::update] Unable to get the jacobian.");
        return m_isValid;
    }

    // A and b are now valid
    m_isValid = true;
    return m_isValid;
}

bool CoMTask::setSetPoint(Eigen::Ref<const Eigen::Vector3d> position,
                          Eigen::Ref<const Eigen::Vector3d> velocity,
                          Eigen::Ref<const Eigen::Vector3d> acceleration)
{
    bool ok = true;
    ok = ok && m_R3Controller.setDesiredState(position, velocity);
    ok = ok && m_R3Controller.setFeedForward(acceleration);
    return ok;
}

std::size_t CoMTask::size() const
{
    return m_linearVelocitySize;
}

CoMTask::Type CoMTask::type() const
{
    return Type::equality;
}

bool CoMTask::isValid() const
{
    return m_isValid;
}
