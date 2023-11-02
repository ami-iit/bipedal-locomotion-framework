/**
 * @file FrictionTorqueStateDynamics.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <Eigen/Dense>

#include <BipedalLocomotion/RobotDynamicsEstimator/FrictionTorqueStateDynamics.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <math.h>

namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

RDE::FrictionTorqueStateDynamics::FrictionTorqueStateDynamics() = default;

RDE::FrictionTorqueStateDynamics::~FrictionTorqueStateDynamics() = default;

bool RDE::FrictionTorqueStateDynamics::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[FrictionTorqueStateDynamics::initialize]";

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

    // Set the list of elements if it exists
    if (!ptr->getParameter("elements", m_elements))
    {
        log()->info("{} Variable elements not found.", errorPrefix);
    }

    // Set the friction parameters
    if (!ptr->getParameter("friction_k0", m_Fc))
    {
        log()->error("{} Error while retrieving the friction_k0 variable.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("friction_k1", m_Fs))
    {
        log()->error("{} Error while retrieving the friction_k1 variable.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("friction_k2", m_Fv))
    {
        log()->error("{} Error while retrieving the friction_k2 variable.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("sampling_time", m_dT))
    {
        log()->error("{} Error while retrieving the sampling_time variable.", errorPrefix);
        return false;
    }

    m_description = "Friction torque state dynamics";

    m_isInitialized = true;

    return true;
}

bool RDE::FrictionTorqueStateDynamics::finalize(const System::VariablesHandler& stateVariableHandler)
{
    constexpr auto errorPrefix = "[FrictionTorqueStateDynamics::finalize]";

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

    m_frictionTorqueFullModel.resize(m_stateVariableHandler.getVariable("tau_F").size);
    m_frictionTorqueFullModel.setZero();

    m_jointVelocityFullModel.resize(m_stateVariableHandler.getVariable("ds").size);
    m_jointVelocityFullModel.setZero();

    m_updatedVariable.resize(m_size);
    m_updatedVariable.setZero();

    m_coshArgument.resize(m_size);
    m_coshArgument.setZero();

    m_coshsquared.resize(m_size);
    m_coshsquared.setZero();

    m_FcFs.resize(m_size);
    m_FcFs.setZero();

    m_dotTauF.resize(m_size);
    m_dotTauF.setZero();

    return true;
}

bool RDE::FrictionTorqueStateDynamics::setSubModels(
    const std::vector<SubModel>& /*subModelList*/,
    const std::vector<std::shared_ptr<KinDynWrapper>>& /*kinDynWrapperList*/)
{
    return true;
}

bool RDE::FrictionTorqueStateDynamics::checkStateVariableHandler()
{
    constexpr auto errorPrefix = "[FrictionTorqueStateDynamics::checkStateVariableHandler]";

    // Check if the variable handler contains the variables used by this dynamics
    if (!m_stateVariableHandler.getVariable("tau_F").isValid())
    {
        log()->error("{} The variable handler does not contain the expected state with name "
                     "`tau_F`.",
                     errorPrefix);
        return false;
    }

    if (!m_stateVariableHandler.getVariable("ds").isValid())
    {
        log()->error("{} The variable handler does not contain the expected state with name `ds`.",
                     errorPrefix);
        return false;
    }

    return true;
}

// TODO
// Change the model
bool RDE::FrictionTorqueStateDynamics::update()
{
    m_coshArgument = m_Fs.array() * m_jointVelocityFullModel.array();

    m_coshsquared = m_coshArgument.array().cosh().array() * m_coshArgument.array().cosh().array();

    m_FcFs = m_Fc.array() * m_Fs.array();

    m_FcFs = (m_FcFs.array() / m_coshsquared.array()).eval();

    m_dotTauF = (m_FcFs + m_Fv).array() * m_ukfInput.robotJointAccelerations.array();

    m_updatedVariable = m_frictionTorqueFullModel.array()
                        + std::chrono::duration<double>(m_dT).count() * m_dotTauF.array();

    return true;
}

void RDE::FrictionTorqueStateDynamics::setState(const Eigen::Ref<const Eigen::VectorXd> ukfState)
{
    m_jointVelocityFullModel = ukfState.segment(m_stateVariableHandler.getVariable("ds").offset,
                                                m_stateVariableHandler.getVariable("ds").size);

    m_frictionTorqueFullModel = ukfState.segment(m_stateVariableHandler.getVariable("tau_F").offset,
                                                 m_stateVariableHandler.getVariable("tau_F").size);
}

void RDE::FrictionTorqueStateDynamics::setInput(const UKFInput& ukfInput)
{
    m_ukfInput = ukfInput;
}
