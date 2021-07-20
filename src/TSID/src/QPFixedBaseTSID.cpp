/**
 * @file QPFixedBaseTSID.cpp
 * @authors Ines Sorrentino, Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/TSID/JointDynamicsTask.h>
#include <BipedalLocomotion/TSID/QPFixedBaseTSID.h>
#include <BipedalLocomotion/TSID/SE3Task.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::TSID;
using namespace BipedalLocomotion::ParametersHandler;

struct QPFixedBaseTSID::Impl
{
    std::string baseLink;
    std::shared_ptr<JointDynamicsTask> dynamicsTask;
    std::shared_ptr<SE3Task> baseSE3Task;
    const std::string baseContactWrenchVariableName{"base_contact_wrench"};
    bool isKinDynSet{false};

    bool createBaseSE3Task(std::shared_ptr<const IParametersHandler> handler)
    {
        constexpr auto logPrefix = "[QPFixedBaseTSID::createBaseSE3Task]";

        auto baseSE3ParameterHandler = std::make_shared<StdImplementation>();
        std::string robotAccelerationVariableName;
        if (!handler->getParameter("robot_acceleration_variable_name",
                                   robotAccelerationVariableName))
        {
            log()->error("{} Unable to get the parameter 'robot_acceleration_variable_name'",
                         logPrefix);
            return false;
        }
        baseSE3ParameterHandler->setParameter("robot_acceleration_variable_name",
                                              robotAccelerationVariableName);

        // This task is to consider the robot fixed base, so the controller gains are set to 0.
        // Read it a zero acceleration task
        baseSE3ParameterHandler->setParameter("kp_linear", 0.0);
        baseSE3ParameterHandler->setParameter("kd_linear", 0.0);
        baseSE3ParameterHandler->setParameter("kp_angular", 0.0);
        baseSE3ParameterHandler->setParameter("kd_angular", 0.0);
        baseSE3ParameterHandler->setParameter("frame_name", baseLink);

        if (!baseSE3Task->initialize(baseSE3ParameterHandler))
        {
            log()->error("{} Error initializing se3task on the base", logPrefix);
            return false;
        }

        baseSE3Task->setSetPoint(manif::SE3d::Identity(),
                                 manif::SE3d::Tangent::Zero(),
                                 manif::SE3d::Tangent::Zero());

        return true;
    }

    bool createDynamicsTask(std::shared_ptr<const IParametersHandler> handler)
    {
        constexpr auto logPrefix = "[QPFixedBaseTSID::createDynamicsTask]";

        auto dynamicsParameterHandler = std::make_shared<StdImplementation>();

        std::string robotAccelerationVariableName;
        if (!handler->getParameter("robot_acceleration_variable_name",
                                   robotAccelerationVariableName))
        {
            log()->error("{} Unable to get the parameter 'robot_acceleration_variable_name'",
                         logPrefix);
            return false;
        }

        std::string jointTorquesVariableName;
        if (!handler->getParameter("joint_torques_variable_name",
                                   jointTorquesVariableName))
        {
            log()->error("{} Unable to get the parameter 'robot_torques_variable_name'",
                         logPrefix);
            return false;
        }

        dynamicsParameterHandler->setParameter("robot_acceleration_variable_name",
                                               robotAccelerationVariableName);
        dynamicsParameterHandler->setParameter("joint_torques_variable_name",
                                               jointTorquesVariableName);

        // We assume the external wrench acting on only the base link.
        dynamicsParameterHandler->setParameter("max_number_of_contacts", 1);

        auto contactGroup = std::make_shared<StdImplementation>();
        contactGroup->setParameter("variable_name", baseContactWrenchVariableName);
        contactGroup->setParameter("frame_name", baseLink);
        if (!dynamicsParameterHandler->setGroup("CONTACT_0", contactGroup))
        {
            log()->error("{} Error setting group for base contact.", logPrefix);
            return false;
        }

        if (!dynamicsTask->initialize(dynamicsParameterHandler))
        {
            log()->error("{} Error initializing dynamics task", logPrefix);
            return false;
        }

        return true;
    }
};

QPFixedBaseTSID::QPFixedBaseTSID()
{
    m_pimpl = std::make_unique<QPFixedBaseTSID::Impl>();

    m_pimpl->baseSE3Task = std::make_shared<SE3Task>();
    m_pimpl->dynamicsTask = std::make_shared<JointDynamicsTask>();
}

QPFixedBaseTSID::~QPFixedBaseTSID() = default;

bool QPFixedBaseTSID::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    constexpr auto logPrefix = "[QPFixedBaseTSID::setKinDyn]";

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

    if (!m_pimpl->dynamicsTask->setKinDyn(kinDyn))
    {
        log()->error("{} Unable to set the kinDyn object for dynamicsTask.", logPrefix);
        return false;
    }

    m_pimpl->baseLink = kinDyn->getFloatingBase();

    m_pimpl->isKinDynSet = true;

    return true;
}

bool QPFixedBaseTSID::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[QPFixedBaseTSID::initialize]";

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

    // the contact_wrench_variables_name parameter is required by QPTSID however if someone uses
    // QPFIxedBaseTSID should not care about it
    auto cloneHandler = ptr->clone();
    cloneHandler->setParameter("contact_wrench_variables_name",
                               std::vector<std::string>{m_pimpl->baseContactWrenchVariableName});

    if (!m_pimpl->createBaseSE3Task(cloneHandler))
    {
        log()->error("{} Error creating SE3task for the base.", logPrefix);
        return false;
    }
    if (!m_pimpl->createDynamicsTask(cloneHandler))
    {
        log()->error("{} Error creating task for the joint dynamics.", logPrefix);
        return false;
    }

    if (!this->QPTSID::initialize(cloneHandler))
    {
        log()->error("{} Unable to initialize QPTSID class.", logPrefix);
        return false;
    }

    constexpr std::size_t highPriority = 0;
    this->addTask(m_pimpl->baseSE3Task, "base_se3_task", highPriority);
    this->addTask(m_pimpl->dynamicsTask, "dynamics_task", highPriority);

    return true;
}

bool QPFixedBaseTSID::finalize(const System::VariablesHandler& handler)
{
    constexpr auto logPrefix = "[QPFixedBaseTSID::finalize]";

    System::VariablesHandler tmpHandler = handler;
    // The vector considering the contact wrench on the base link contains 6 elements, the 3 forces
    // in the 3 directions and the 3 moments about the 3 axes.
    constexpr auto wrenchSize = BipedalLocomotion::Math::Wrenchd::SizeAtCompileTime;
    if (!tmpHandler.addVariable(m_pimpl->baseContactWrenchVariableName, wrenchSize))
    {
        log()->error("{} Error while adding base contact wrench variable to the handler",
                     logPrefix);
        return false;
    }

    return this->QPTSID::finalize(tmpHandler);
}
