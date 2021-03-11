/**
 * @file CoMTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/IK/CoMTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;

bool CoMTask::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    if (!m_isInitialized)
    {
        log()->error("[CoMTask::setVariablesHandler] The task is not initialized. Please call "
                     "initialize method.");
        return false;
    }

    // get the variable
    if (!variablesHandler.getVariable(m_robotVelocityVariable.name, m_robotVelocityVariable))
    {
        log()->error("[CoMTask::setVariablesHandler] Unable to get the variable named {}.",
                     m_robotVelocityVariable.name);
        return false;
    }

    // get the variable
    if (m_robotVelocityVariable.size != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        log()->error("[CoMTask::setVariablesHandler] The size of the robot velocity variable is "
                     "different from the one expected. Expected size: {}. Given size: {}.",
                     m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize,
                     m_robotVelocityVariable.size);
        return false;
    }

    // resize the matrices
    m_A.resize(m_linearVelocitySize, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_linearVelocitySize);
    m_b.setZero();

    return true;
}

bool CoMTask::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[CoMTask::initialize]";

    m_description = "CoMTask Optimal Control Element";

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
    double kpLinear;
    if (!ptr->getParameter("kp_linear", kpLinear))
    {
        log()->error("{} [{}] to get the proportional linear gain.", errorPrefix, m_description);
        return false;
    }

    m_R3Controller.setGains(kpLinear);

    m_isInitialized = true;

    return true;
}

bool CoMTask::update()
{
    using namespace BipedalLocomotion::Conversions;
    using namespace iDynTree;

    // set the state
    m_R3Controller.setState(toEigen(m_kinDyn->getCenterOfMassPosition()));

    // update the controller
    m_R3Controller.computeControlLaw();

    m_b = m_R3Controller.getControl().coeffs();

    // get the CoM jacobian
    if (!m_kinDyn->getCenterOfMassJacobian(this->subA(m_robotVelocityVariable)))
    {
        log()->error("[CoMTask::update] Unable to get the jacobian.");
        return false;
    }

    return true;
}

bool CoMTask::setSetPoint(Eigen::Ref<const Eigen::Vector3d> position,
                          Eigen::Ref<const Eigen::Vector3d> velocity)
{
    bool ok = true;
    ok = ok && m_R3Controller.setDesiredState(position);
    ok = ok && m_R3Controller.setFeedForward(velocity);

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
