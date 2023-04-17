/**
 * @file JointVelocityStateDynamics.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <map>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/JointVelocityStateDynamics.h>

namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

RDE::JointVelocityStateDynamics::JointVelocityStateDynamics() = default;

RDE::JointVelocityStateDynamics::~JointVelocityStateDynamics() = default;

bool RDE::JointVelocityStateDynamics::setSubModels(const std::vector<RDE::SubModel>& subModelList, const std::vector<std::shared_ptr<RDE::SubModelKinDynWrapper>>& kinDynWrapperList)
{
    return true;
}

bool RDE::JointVelocityStateDynamics::checkStateVariableHandler()
{
    constexpr auto errorPrefix = "[JointVelocityStateDynamics::checkStateVariableHandler]";

    // Check if the variable handler contains the variables used by this dynamics
    if (!m_stateVariableHandler.getVariable("tau_m").isValid())
    {
        log()->error("{} The variable handler does not contain the expected state with name `tau_m`.", errorPrefix);
        return false;
    }

    if (!m_stateVariableHandler.getVariable("tau_F").isValid())
    {
        log()->error("{} The variable handler does not contain the expected state with name `tau_F`.", errorPrefix);
        return false;
    }

    return true;
}

bool RDE::JointVelocityStateDynamics::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[JointVelocityStateDynamics::initialize]";

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", errorPrefix);
        return false;
    }

    // Set the state dynamics name
    if (!ptr->getParameter("name", m_name))
    {
        log()->error("{} Error while retrieving the name variable.", errorPrefix);
        return false;
    }

    // Set the state process covariance
    if (!ptr->getParameter("covariance", m_covariances))
    {
        log()->error("{} Error while retrieving the covariance variable.", errorPrefix);
        return false;
    }

    // Set the state initial covariance
    if (!ptr->getParameter("initial_covariance", m_initialCovariances))
    {
        log()->error("{} Error while retrieving the initial_covariance variable.", errorPrefix);
        return false;
    }

    // Set the dynamic model type
    if (!ptr->getParameter("dynamic_model", m_dynamicModel))
    {
        log()->error("{} Error while retrieving the dynamic_model variable.", errorPrefix);
        return false;
    }

    // Set the list of elements if it exists
    if (!ptr->getParameter("elements", m_elements))
    {
        log()->info("{} Variable elements not found.", errorPrefix);
        m_elements = {};
    }

    if (!ptr->getParameter("sampling_time", m_dT))
    {
        log()->info("{} Error while retrieving the sampling_time variable.", errorPrefix);
        m_elements = {};
    }

    m_description = "Joint velocity state dynamics depending on the robot dynamic model";

    m_isInitialized = true;

    return true;
}

bool RDE::JointVelocityStateDynamics::finalize(const System::VariablesHandler &stateVariableHandler)
{
    constexpr auto errorPrefix = "[JointVelocityStateDynamics::finalize]";

    if (!m_isInitialized)
    {
        log()->error("{} Please initialize the dynamics before calling finalize.", errorPrefix);
        return false;
    }

    if (stateVariableHandler.getNumberOfVariables() == 0)
    {
        log()->error("{} The state variable handler is empty.", errorPrefix);
        return false;
    }

    m_stateVariableHandler = stateVariableHandler;

    if (!checkStateVariableHandler())
    {
        log()->error("{} The state variable handler is not valid.", errorPrefix);
        return false;
    }

    m_size = m_covariances.size();

    m_jointVelocityFullModel.resize(m_stateVariableHandler.getVariable("ds").size);
    m_jointVelocityFullModel.setZero();

    m_updatedVariable.resize(m_stateVariableHandler.getVariable("ds").size);
    m_updatedVariable.setZero();

    return true;
}


bool RDE::JointVelocityStateDynamics::update()
{
    m_updatedVariable = m_jointVelocityFullModel + m_dT * m_ukfInput.robotJointAccelerations;

    return true;
}

void RDE::JointVelocityStateDynamics::setState(const Eigen::Ref<const Eigen::VectorXd> ukfState)
{
    m_jointVelocityFullModel = ukfState.segment(m_stateVariableHandler.getVariable("ds").offset,
                                                m_stateVariableHandler.getVariable("ds").size);
}

void RDE::JointVelocityStateDynamics::setInput(const UKFInput& ukfInput)
{
    m_ukfInput = ukfInput;
}
