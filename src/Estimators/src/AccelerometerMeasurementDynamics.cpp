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
    for (int submodelIndex = 0; submodelIndex < m_nrOfSubDynamics; submodelIndex++)
    {
       if (m_subDynamics[submodelIndex]->subModel.hasAccelerometer(m_name))
       {
            m_subModelsWithAcc.push_back(submodelIndex);
       }
    }

    m_covariances.resize(m_covSingleVar.size() * m_subModelsWithAcc.size());
    for (int index = 0; index < m_subModelsWithAcc.size(); index++)
    {
        m_covariances.segment(index*m_covSingleVar.size(), m_covSingleVar.size()) = m_covSingleVar;
    }

    m_size = m_covariances.size();

    m_motorTorqueFullModel.resize(m_stateVariableHandler.getVariable("tau_m").size);
    m_motorTorqueFullModel.setZero();

    m_frictionTorqueFullModel.resize(m_stateVariableHandler.getVariable("tau_F").size);
    m_frictionTorqueFullModel.setZero();

    m_jointAccelerationFullModel.resize(m_stateVariableHandler.getVariable("ds").size);
    m_jointAccelerationFullModel.setZero();

    m_jointVelocityFullModel.resize(m_stateVariableHandler.getVariable("ds").size);
    m_jointVelocityFullModel.setZero();

    m_subModelJointAcc.resize(m_nrOfSubDynamics);

    for (int idx = 0; idx < m_nrOfSubDynamics; idx++)
    {
        m_subModelJointAcc[idx].resize(m_subDynamics[idx]->subModel.getJointMapping().size());
        m_subModelJointAcc[idx].setZero();
    }

    m_bias.resize(m_covSingleVar.size());
    m_bias.setZero();

    m_updatedVariable.resize(m_size);
    m_updatedVariable.setZero();

    return true;
}

bool RDE::AccelerometerMeasurementDynamics::setSubModels(const std::vector<SubModel>& subModelList, const std::vector<std::shared_ptr<SubModelKinDynWrapper>>& kinDynWrapperList)
{
    constexpr auto errorPrefix = "[AccelerometerMeasurementDynamics::setSubModels]";

    for (int subModelIdx = 0; subModelIdx < subModelList.size(); subModelIdx++)
    {
        m_subDynamics.emplace_back(std::make_unique<RDE::SubModelDynamics>());

        if(!m_subDynamics[subModelIdx]->setSubModel(subModelList[subModelIdx]))
        {
            log()->error("{} The submodel at index {} is not valid.", errorPrefix, subModelIdx);
            return false;
        }

        if(!m_subDynamics[subModelIdx]->setKinDynWrapper(kinDynWrapperList[subModelIdx]))
        {
            log()->error("{} The submodel kindyn wrapper at index {} is not valid.", errorPrefix, subModelIdx);
            return false;
        }
    }

    m_nrOfSubDynamics = m_subDynamics.size();

    m_isSubModelListSet = true;

    for (int subModelIdx = 0; subModelIdx < m_nrOfSubDynamics; subModelIdx++)
    {
        if (!m_subDynamics.at(subModelIdx)->initialize())
        {
            log()->error("{} Cannot initialize the accelerometer measurement dynamics of the sub-model {}.", errorPrefix, subModelIdx);
            return false;
        }
    }

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
    for (int subModelIdx = 0; subModelIdx < m_nrOfSubDynamics; subModelIdx++)
    {
        for (int ftIdx = 0; ftIdx < m_subDynamics.at(subModelIdx)->subModel.getNrOfFTSensor(); ftIdx++)
        {
            if (!m_stateVariableHandler.getVariable(m_subDynamics.at(subModelIdx)->subModel.getFTSensor(ftIdx).name).isValid())
            {
                log()->error("{} The variable handler does not contain the expected state with name `{}`.", errorPrefix, m_subDynamics.at(subModelIdx)->subModel.getFTSensor(ftIdx).name);
                return false;
            }
        }

        for (int contactIdx = 0; contactIdx < m_subDynamics.at(subModelIdx)->subModel.getNrOfExternalContact(); contactIdx++)
        {
            if (!m_stateVariableHandler.getVariable(m_subDynamics.at(subModelIdx)->subModel.getExternalContact(contactIdx)).isValid())
            {
                log()->error("{} The variable handler does not contain the expected state with name `{}`.", errorPrefix, m_subDynamics.at(subModelIdx)->subModel.getExternalContact(contactIdx));
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
    for (int subDynamicsIdx = 0; subDynamicsIdx < m_subDynamics.size(); subDynamicsIdx++)
    {
        if (m_subDynamics[subDynamicsIdx]->subModel.getModel().getNrOfDOFs() > 0)
        {
            if (!m_subDynamics[subDynamicsIdx]->update(m_ukfInput.robotBaseAcceleration, m_jointAccelerationFullModel, m_subModelJointAcc[subDynamicsIdx]))
            {
                log()->error("{} Error updating the joint velocity dynamics for the sub-model {}.", errorPrefix, subDynamicsIdx);
                return false;
            }

            for (int jointIdx = 0; jointIdx < m_subDynamics[subDynamicsIdx]->subModel.getJointMapping().size(); jointIdx++)
            {
                m_jointAccelerationFullModel[m_subDynamics[subDynamicsIdx]->subModel.getJointMapping()[jointIdx]] = m_subModelJointAcc[subDynamicsIdx][jointIdx];
            }
        }
    }

    // J_dot nu + base_J v_dot_base + joint_J s_dotdot - acc_Rot_world gravity + bias
    for (int index = 0; index < m_subModelsWithAcc.size(); index++)
    {
        m_JdotNu = m_subDynamics[m_subModelsWithAcc[index]]->kinDynWrapper->getAccelerometerBiasAcceleration(m_name);

        if (!m_subDynamics[m_subModelsWithAcc[index]]->kinDynWrapper->getBaseAcceleration(m_ukfInput.robotBaseAcceleration, m_jointAccelerationFullModel, m_subModelBaseAcc))
        {
            log()->error("{} Error getting the sub-model base acceleration.", errorPrefix);
            return false;
        }

        m_JvdotBase = m_subDynamics[m_subModelsWithAcc[index]]->kinDynWrapper->getAccelerometerJacobian(m_name).block(0, 0, 6, 6) * m_subModelBaseAcc.coeffs();

        m_accRg = m_subDynamics[m_subModelsWithAcc[index]]->kinDynWrapper->getAccelerometerRotation(m_name).rotation() * m_gravity;

        m_updatedVariable.segment(index * m_covSingleVar.size(), m_covSingleVar.size()) = m_JdotNu.segment(0, 3) + m_JvdotBase.segment(0, 3) - m_accRg + m_bias;

        if (m_subDynamics[m_subModelsWithAcc[index]]->subModel.getJointMapping().size() > 0)
        {
            m_Jsdotdot = m_subDynamics[m_subModelsWithAcc[index]]->kinDynWrapper->getAccelerometerJacobian(m_name).block(0, 6, 6, m_subModelJointAcc[m_subModelsWithAcc[index]].size()) *
                    m_subModelJointAcc[m_subModelsWithAcc[index]];

            m_updatedVariable.segment(index * m_covSingleVar.size(), m_covSingleVar.size()) += m_Jsdotdot.segment(0, 3);
        }
    }

    return true;
}

void RDE::AccelerometerMeasurementDynamics::setState(const Eigen::Ref<const Eigen::VectorXd> ukfState)
{
    m_jointVelocityFullModel = ukfState.segment(m_stateVariableHandler.getVariable("ds").offset,
                                                m_stateVariableHandler.getVariable("ds").size);

    m_motorTorqueFullModel = ukfState.segment(m_stateVariableHandler.getVariable("tau_m").offset,
                                              m_stateVariableHandler.getVariable("tau_m").size);

    m_frictionTorqueFullModel = ukfState.segment(m_stateVariableHandler.getVariable("tau_F").offset,
                                                 m_stateVariableHandler.getVariable("tau_F").size);

    m_bias = ukfState.segment(m_stateVariableHandler.getVariable(m_biasVariableName).offset,
                              m_stateVariableHandler.getVariable(m_biasVariableName).size);

    for (int subDynamicsIdx = 0; subDynamicsIdx < m_subDynamics.size(); subDynamicsIdx++)
    {
        m_subDynamics.at(subDynamicsIdx)->setState(ukfState,
                                                   m_jointVelocityFullModel,
                                                   m_motorTorqueFullModel,
                                                   m_frictionTorqueFullModel,
                                                   m_stateVariableHandler);
    }
}

void RDE::AccelerometerMeasurementDynamics::setInput(const UKFInput& ukfInput)
{
    m_ukfInput = ukfInput;
}
