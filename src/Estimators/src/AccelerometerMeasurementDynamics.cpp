/**
 * @file AccelerometerMeasurementDynamics.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <Eigen/Dense>

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/AccelerometerMeasurementDynamics.h>

namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

RDE::AccelerometerMeasurementDynamics::AccelerometerMeasurementDynamics() = default;

RDE::AccelerometerMeasurementDynamics::~AccelerometerMeasurementDynamics() = default;

bool RDE::AccelerometerMeasurementDynamics::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[AccelerometerMeasurementDynamics::initialize]";

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
    if (!ptr->getParameter("covariance", m_covSingleVar))
    {
        log()->error("{} Error while retrieving the covariance variable.", errorPrefix);
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
        log()->error("{} Error while retrieving the sampling_time variable.", errorPrefix);
        return false;
    }

    // Set the bias related variables if use_bias is true
    if (!ptr->getParameter("use_bias", m_useBias))
    {
        log()->info("{} Variable use_bias not found. Set to false by default.", errorPrefix);
    }
    else
    {
        m_useBias = true;
        m_biasVariableName = m_name + "_bias";
    }

    m_gravity.setZero();
    m_gravity[2] = - BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    m_JdotNu.resize(6);
    m_JvdotBase.resize(6);
    m_Jsdotdot.resize(6);

    m_description = "Accelerometer measurement dynamics";

    m_isInitialized = true;

    return true;
}

bool RDE::AccelerometerMeasurementDynamics::finalize(const System::VariablesHandler &stateVariableHandler)
{
    constexpr auto errorPrefix = "[AccelerometerMeasurementDynamics::finalize]";

    if (!m_isInitialized)
    {
        log()->error("{} Please initialize the dynamics before calling finalize.", errorPrefix);
        return false;
    }

    if (!m_isSubModelListSet)
    {
        log()->error("{} Please call `setSubModels` before finalizing.", errorPrefix);
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

    // Search and save all the submodels containing the sensor
    for (int submodelIndex = 0; submodelIndex < m_subModelList.size(); submodelIndex++)
    {
        if (m_subModelList[submodelIndex].hasAccelerometer(m_name))
        {
            m_subModelsWithAccelerometer.push_back(submodelIndex);
        }
    }

    m_covariances.resize(m_covSingleVar.size() * m_subModelsWithAccelerometer.size());
    for (int index = 0; index < m_subModelsWithAccelerometer.size(); index++)
    {
        m_covariances.segment(index*m_covSingleVar.size(), m_covSingleVar.size()) = m_covSingleVar;
    }

    m_size = m_covariances.size();

    m_subModelJointAcc.resize(m_subModelList.size());

    for (int idx = 0; idx < m_subModelList.size(); idx++)
    {
        m_subModelJointAcc[idx].resize(m_subModelList[idx].getJointMapping().size());
        m_subModelJointAcc[idx].setZero();
    }

    m_bias.resize(m_covSingleVar.size());
    m_bias.setZero();

    m_updatedVariable.resize(m_size);
    m_updatedVariable.setZero();

    m_accFrameVel.setZero();

    return true;
}

bool RDE::AccelerometerMeasurementDynamics::setSubModels(const std::vector<SubModel>& subModelList, const std::vector<std::shared_ptr<SubModelKinDynWrapper>>& kinDynWrapperList)
{
    constexpr auto errorPrefix = "[AccelerometerMeasurementDynamics::setSubModels]";

    m_subModelList = subModelList;

    m_kinDynWrapperList = kinDynWrapperList;

    m_isSubModelListSet = true;

    return true;
}

bool RDE::AccelerometerMeasurementDynamics::checkStateVariableHandler()
{
    constexpr auto errorPrefix = "[AccelerometerMeasurementDynamics::checkStateVariableHandler]";

    if (!m_isSubModelListSet)
    {
        log()->error("{} Set the sub-model list before setting the variable handler", errorPrefix);
        return false;
    }

    if (!m_stateVariableHandler.getVariable("tau_m").isValid())
    {
        log()->error("{} The variable handler does not contain the expected state with name `tau_m`.", errorPrefix);
        return false;
    }

    // Check if the variable handler contains the variables used by this dynamics
    if (!m_stateVariableHandler.getVariable("tau_F").isValid())
    {
        log()->error("{} The variable handler does not contain the expected state with name `tau_F`.", errorPrefix);
        return false;
    }

    if (!m_stateVariableHandler.getVariable("ds").isValid())
    {
        log()->error("{} The variable handler does not contain the expected state with name `ds`.", errorPrefix);
        return false;
    }

    if (m_useBias)
    {
        if (!m_stateVariableHandler.getVariable(m_biasVariableName).isValid())
        {
            log()->error("{} The variable handler does not contain the expected state with name `{}`.", errorPrefix, m_biasVariableName);
            return false;
        }
    }

    // check if all the sensors are part of the sub-model
    for (int subModelIdx = 0; subModelIdx < m_subModelList.size(); subModelIdx++)
    {
        for (int ftIdx = 0; ftIdx < m_subModelList[subModelIdx].getNrOfFTSensor(); ftIdx++)
        {
            if (!m_stateVariableHandler.getVariable(m_subModelList[subModelIdx].getFTSensor(ftIdx).name).isValid())
            {
                log()->error("{} The variable handler does not contain the expected state with name `{}`.", errorPrefix, m_subModelList[subModelIdx].getFTSensor(ftIdx).name);
                return false;
            }
        }

        for (int contactIdx = 0; contactIdx < m_subModelList[subModelIdx].getNrOfExternalContact(); contactIdx++)
        {
            if (!m_stateVariableHandler.getVariable(m_subModelList[subModelIdx].getExternalContact(contactIdx)).isValid())
            {
                log()->error("{} The variable handler does not contain the expected state with name `{}`.", errorPrefix, m_subModelList[subModelIdx].getExternalContact(contactIdx));
                return false;
            }
        }
    }

    return true;
}

bool RDE::AccelerometerMeasurementDynamics::update()
{
    constexpr auto errorPrefix = "[AccelerometerMeasurementDynamics::update]";

    // compute joint acceleration per each sub-model containing the accelerometer
    for (int subDynamicsIdx = 0; subDynamicsIdx < m_subModelList.size(); subDynamicsIdx++)
    {
        if (m_subModelList[subDynamicsIdx].getModel().getNrOfDOFs() > 0)
        {
            for (int jointIdx = 0; jointIdx < m_subModelList[subDynamicsIdx].getJointMapping().size(); jointIdx++)
            {
                m_subModelJointAcc[subDynamicsIdx][jointIdx] = m_ukfInput.robotJointAccelerations[m_subModelList[subDynamicsIdx].getJointMapping()[jointIdx]];
            }
        }
    }

    // J_dot nu + base_J v_dot_base + joint_J s_dotdot - acc_Rot_world gravity + bias
    for (int index = 0; index < m_subModelsWithAccelerometer.size(); index++)
    {
        m_JdotNu = m_kinDynWrapperList[m_subModelsWithAccelerometer[index]]->getAccelerometerBiasAcceleration(m_name);

        m_JvdotBase = m_kinDynWrapperList[m_subModelsWithAccelerometer[index]]->getAccelerometerJacobian(m_name).block(0, 0, 6, 6) *
                m_kinDynWrapperList[m_subModelsWithAccelerometer[index]]->getBaseAcceleration().coeffs();

        m_accRg = m_kinDynWrapperList[m_subModelsWithAccelerometer[index]]->getAccelerometerRotation(m_name).rotation() * m_gravity;

        linVel = m_kinDynWrapperList[m_subModelsWithAccelerometer[index]]->getAccelerometerVelocity(m_name).coeffs().segment(0, 3);
        angVel = m_kinDynWrapperList[m_subModelsWithAccelerometer[index]]->getAccelerometerVelocity(m_name).coeffs().segment(3, 6);

        m_vCrossW = linVel.cross(angVel);

        m_updatedVariable.segment(index * m_covSingleVar.size(), m_covSingleVar.size()) = m_JdotNu.segment(0, 3) + m_JvdotBase.segment(0, 3) - m_vCrossW - m_accRg + m_bias;

        if (m_subModelList[m_subModelsWithAccelerometer[index]].getJointMapping().size() > 0)
        {
            m_Jsdotdot = m_kinDynWrapperList[m_subModelsWithAccelerometer[index]]->getAccelerometerJacobian(m_name).
                    block(0, 6, 6, m_subModelJointAcc[m_subModelsWithAccelerometer[index]].size()) *
                    m_subModelJointAcc[m_subModelsWithAccelerometer[index]];

            m_updatedVariable.segment(index * m_covSingleVar.size(), m_covSingleVar.size()) += m_Jsdotdot.segment(0, 3);
        }
    }

    return true;
}

void RDE::AccelerometerMeasurementDynamics::setState(const Eigen::Ref<const Eigen::VectorXd> ukfState)
{
    m_bias = ukfState.segment(m_stateVariableHandler.getVariable(m_biasVariableName).offset,
                              m_stateVariableHandler.getVariable(m_biasVariableName).size);
}

void RDE::AccelerometerMeasurementDynamics::setInput(const UKFInput& ukfInput)
{
    m_ukfInput = ukfInput;
}
