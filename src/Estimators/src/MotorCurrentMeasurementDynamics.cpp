/**
 * @file MotorCurrentMeasurementDynamics.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <Eigen/Dense>

#include <BipedalLocomotion/RobotDynamicsEstimator/MotorCurrentMeasurementDynamics.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

RDE::MotorCurrentMeasurementDynamics::MotorCurrentMeasurementDynamics() = default;

RDE::MotorCurrentMeasurementDynamics::~MotorCurrentMeasurementDynamics() = default;

bool RDE::MotorCurrentMeasurementDynamics::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[MotorCurrentMeasurementDynamics::initialize]";

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
    if (!ptr->getParameter("covariance", this->m_covariances))
    {
        log()->error("{} Error while retrieving the covariance variable.", errorPrefix);
        return false;
    }

    // Set the list of elements if it exists
    if (!ptr->getParameter("elements", m_elements))
    {
        log()->info("{} Variable elements not found.", errorPrefix);
    }

    // Set the torque constants
    if (!ptr->getParameter("torque_constant", m_kTau))
    {
        log()->error("{} Error while retrieving the torque_constant variable.", errorPrefix);
        return false;
    }

    // Set the gearbox reduction ratio
    if (!ptr->getParameter("gear_ratio", m_gearRatio))
    {
        log()->error("{} Error while retrieving the gear_ratio variable.", errorPrefix);
        return false;
    }

    m_description = "Motor current measurement dynamics";

    m_isInitialized = true;

    return true;
}

bool RDE::MotorCurrentMeasurementDynamics::finalize(
    const System::VariablesHandler& stateVariableHandler)
{
    constexpr auto errorPrefix = "[MotorCurrentMeasurementDynamics::finalize]";

    if (!m_isInitialized)
    {
        log()->error("{} Please initialize the dynamics before calling finalize.", errorPrefix);
        return false;
    }

    if (stateVariableHandler.getNumberOfVariables() == 0)
    {
        log()->error("{} The variable handler is empty.", errorPrefix);
        return false;
    }

    m_stateVariableHandler = stateVariableHandler;

    if (!checkStateVariableHandler())
    {
        log()->error("{} The variable handler is not valid.", errorPrefix);
        return false;
    }

    m_size = m_covariances.size();

    m_updatedVariable.resize(m_size);
    m_updatedVariable.setZero();

    m_motorTorque.resize(m_size);
    m_motorTorque.setZero();

    return true;
}

bool RDE::MotorCurrentMeasurementDynamics::setSubModels(
    const std::vector<RDE::SubModel>& /*subModelList*/,
    const std::vector<std::shared_ptr<RDE::KinDynWrapper>>& /*kinDynWrapperList*/)
{
    return true;
}

bool RDE::MotorCurrentMeasurementDynamics::checkStateVariableHandler()
{
    constexpr auto errorPrefix = "[MotorCurrentMeasurementDynamics::checkStateVariableHandler]";

    // Check if the variable handler contains the variables used by this dynamics
    if (!m_stateVariableHandler.getVariable("tau_m").isValid())
    {
        log()->error("{} The variable handler does not contain the expected measurement name {}.",
                     errorPrefix,
                     "tau_m");
        return false;
    }

    return true;
}

bool RDE::MotorCurrentMeasurementDynamics::update()
{
    m_updatedVariable = m_motorTorque.array() / (m_kTau.array() * m_gearRatio.array());

    return true;
}

void RDE::MotorCurrentMeasurementDynamics::setState(const Eigen::Ref<const Eigen::VectorXd> ukfState)
{
    m_motorTorque = ukfState.segment(m_stateVariableHandler.getVariable("tau_m").offset,
                                     m_stateVariableHandler.getVariable("tau_m").size);
}

void RDE::MotorCurrentMeasurementDynamics::setInput(const UKFInput& /*ukfInput*/)
{
    return;
}
