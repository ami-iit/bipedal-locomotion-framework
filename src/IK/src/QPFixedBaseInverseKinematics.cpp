/**
 * @file QPFixedBaseInverseKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/IK/QPFixedBaseInverseKinematics.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::IK;
using namespace BipedalLocomotion::ParametersHandler;

struct QPFixedBaseInverseKinematics::Impl
{
    std::string baseLink;
    std::shared_ptr<SE3Task> baseSE3Task;
    // const std::string baseContactWrenchVariableName{"base_contact_wrench"};
    bool isKinDynSet{false};

    bool createBaseSE3Task(std::shared_ptr<const IParametersHandler> handler)
    {
        constexpr auto logPrefix = "[QPFixedBaseInverseKinematics::createBaseSE3Task]";

        auto baseSE3ParameterHandler = std::make_shared<StdImplementation>();
        std::string robotVelocityVariableName;
        if (!handler->getParameter("robot_velocity_variable_name", robotVelocityVariableName))
        {
            log()->error("{} Unable to get the parameter 'robot_velocity_variable_name'",
                         logPrefix);
            return false;
        }
        baseSE3ParameterHandler->setParameter("robot_velocity_variable_name",
                                              robotVelocityVariableName);

        // This task is to consider the robot fixed base, so the controller gains are set to 0.
        // Read it a zero acceleration task
        baseSE3ParameterHandler->setParameter("kp_linear", 0.0);
        baseSE3ParameterHandler->setParameter("kp_angular", 0.0);
        baseSE3ParameterHandler->setParameter("frame_name", this->baseLink);

        if (!baseSE3Task->initialize(baseSE3ParameterHandler))
        {
            log()->error("{} Error initializing se3task on the base", logPrefix);
            return false;
        }

        baseSE3Task->setTaskControllerMode(SE3Task::Mode::Disable);
        baseSE3Task->setSetPoint(manif::SE3d::Identity(),
                                 manif::SE3d::Tangent::Zero());

        return true;
    }
};

QPFixedBaseInverseKinematics::QPFixedBaseInverseKinematics()
{
    m_pimpl = std::make_unique<QPFixedBaseInverseKinematics::Impl>();
    m_pimpl->baseSE3Task = std::make_shared<SE3Task>();
}

QPFixedBaseInverseKinematics::~QPFixedBaseInverseKinematics() = default;

bool QPFixedBaseInverseKinematics::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    constexpr auto logPrefix = "[QPFixedBaseInverseKinematics::setKinDyn]";

    if (kinDyn == nullptr)
    {
        log()->error("{} The kinDyn object is not valid.", logPrefix);
        return false;
    }

    if (!m_pimpl->baseSE3Task->setKinDyn(kinDyn))
    {
        log()->error("{} Unable to set the kinDyn object for SE3Task on the base.", logPrefix);
        return false;
    }

    // get the robot base link
    m_pimpl->baseLink = kinDyn->getFloatingBase();

    m_pimpl->isKinDynSet = true;

    return true;
}

bool QPFixedBaseInverseKinematics::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[QPFixedBaseInverseKinematics::initialize]";

    if (!m_pimpl->isKinDynSet)
    {
        log()->error("{} The object kinDyn is not valid.", logPrefix);
        return false;
    }

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", logPrefix);
        return false;
    }

    if (!m_pimpl->createBaseSE3Task(ptr))
    {
        log()->error("{} Error creating SE3task for the base.", logPrefix);
        return false;
    }

    if (!this->QPInverseKinematics::initialize(ptr))
    {
        log()->error("{} Unable to initialize QPInverseKinematics class.", logPrefix);
        return false;
    }

    constexpr std::size_t highPriority = 0;
    return this->addTask(m_pimpl->baseSE3Task, "base_se3_task", highPriority);
}
