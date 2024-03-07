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

    auto setGain
        = [logPrefix, ptr](const std::string& gainName) -> std::shared_ptr<System::WeightProvider> {
        auto group = ptr->getGroup(gainName).lock();
        if (group == nullptr)
        {
            log()->error("{} Unable to find the group {}.", logPrefix, gainName);
            return nullptr;
        }

        std::string weightProviderType = "ConstantWeightProvider";
        if (!group->getParameter("gain_provider_type", weightProviderType))
        {
            log()->warn("{} Unable to find the parameter 'gain_provider_type' in the group {}. The "
                        "default value {} is used.",
                        logPrefix,
                        gainName,
                        weightProviderType);
        }

        auto weightProvider = System::WeightProviderFactory::createInstance(weightProviderType);

        if (weightProvider == nullptr)
        {
            log()->error("{} Unable to create the weight provider for the {}.",
                         logPrefix,
                         gainName);
            return nullptr;
        }

        if (!weightProvider->initialize(group))
        {
            log()->error("{} Unable to initialize the weight provider for the {}.",
                         logPrefix,
                         gainName);
            return nullptr;
        }

        return weightProvider;
    };

    m_CoMGainProvider = setGain("COM_GAIN"); // CoM gain is the gain used to control the CoM
                                             // position

    if (m_CoMGainProvider == nullptr)
    {
        log()->error("{} Unable to create the weight provider for the com_gain.", logPrefix);
        return false;
    }

    m_ZMPGainProvider = setGain("ZMP_GAIN"); // ZMP gain is the gain used to control the ZMP
                                             // position
    if (m_ZMPGainProvider == nullptr)
    {
        log()->error("{} Unable to create the weight provider for the zmp_gain.", logPrefix);
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
    m_controllerOutput.noalias()
        += m_I_R_B.act(m_CoMGainProvider->getOutput().asDiagonal()
                       * m_I_R_B.inverse().act(m_desiredCoMPosition - m_CoMPosition));

    // ZMP Controller
    m_controllerOutput.noalias()
        += m_I_R_B.act(m_ZMPGainProvider->getOutput().asDiagonal()
                       * m_I_R_B.inverse().act(m_ZMPPosition - m_desiredZMPPosition));

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

std::shared_ptr<System::WeightProvider> CoMZMPController::getCoMGainProvider() const
{
    return m_CoMGainProvider;
}

std::shared_ptr<System::WeightProvider> CoMZMPController::getZMPGainProvider() const
{
    return m_ZMPGainProvider;
}
