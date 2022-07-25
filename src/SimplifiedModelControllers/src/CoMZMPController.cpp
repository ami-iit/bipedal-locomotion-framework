/**
 * @file CoMZMPController.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/SimplifiedModelControllers/CoMZMPController.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <memory>

using namespace BipedalLocomotion::SimplifiedModelControllers;
using namespace BipedalLocomotion;

bool CoMZMPController::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[CoMZMPController::initialize]";

    m_isOutputValid = false;

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The handler has to point to an already initialized IParametershandler.",
                     logPrefix);
        return false;
    }

    bool ok = ptr->getParameter("com_gain", m_CoMGain);
    ok = ok && ptr->getParameter("zmp_gain", m_ZMPGain);

    if (!ok)
    {
        log()->error("{} Unable to load the controller gains.", logPrefix);
        return false;
    }

    m_isInitalized = true;

    return true;
}

const CoMZMPController::Output& CoMZMPController::getOutput() const
{
    return m_controllerOutput;
}

bool CoMZMPController::isOutputValid() const
{
    return m_isOutputValid;
}

bool CoMZMPController::advance()
{
    if (!m_isInitalized)
    {
        log()->error("[CoMZMPController::advance] The controller is not initialized. Please call "
                     "the 'initialize()' method");
        return false;
    }

    if (m_isOutputValid)
        return true;

    // feed forward
    m_controllerOutput = m_desiredCoMVelocity;

    // CoM Controller
    m_controllerOutput.noalias() += m_I_R_B.act(
        m_CoMGain.asDiagonal() * m_I_R_B.inverse().act(m_desiredCoMPosition - m_CoMPosition));

    // ZMP Controller
    m_controllerOutput.noalias() += m_I_R_B.act(
        m_ZMPGain.asDiagonal() * m_I_R_B.inverse().act(m_ZMPPosition - m_desiredZMPPosition));

    m_isOutputValid = true;

    return true;
}

bool CoMZMPController::setInput(const Input& input)
{
    this->setFeedback(input.CoMPosition, input.ZMPPosition, input.angle);
    this->setSetPoint(input.desiredCoMVelocity, input.desiredCoMPosition, input.desiredZMPPosition);

    return true;
}

void CoMZMPController::setSetPoint(Eigen::Ref<const Eigen::Vector2d> CoMVelocity,
                                   Eigen::Ref<const Eigen::Vector2d> CoMPosition,
                                   Eigen::Ref<const Eigen::Vector2d> ZMPPosition)
{

    m_desiredCoMVelocity = CoMVelocity;
    m_desiredCoMPosition = CoMPosition;
    m_desiredZMPPosition = ZMPPosition;

    m_isOutputValid = false;
}

void CoMZMPController::setFeedback(Eigen::Ref<const Eigen::Vector2d> CoMPosition,
                                   Eigen::Ref<const Eigen::Vector2d> ZMPPosition,
                                   const manif::SO2d& I_R_B)
{
    m_CoMPosition = CoMPosition;
    m_ZMPPosition = ZMPPosition;
    m_I_R_B = I_R_B;

    m_isOutputValid = false;
}

void CoMZMPController::setFeedback(Eigen::Ref<const Eigen::Vector2d> CoMPosition,
                                   Eigen::Ref<const Eigen::Vector2d> ZMPPosition,
                                   const double angle)
{
    this->setFeedback(CoMPosition, ZMPPosition, manif::SO2d(angle));
}
