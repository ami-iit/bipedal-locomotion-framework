/**
 * @file GyroscopeMeasurementDynamics.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <Eigen/Dense>

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/GyroscopeMeasurementDynamics.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

RDE::GyroscopeMeasurementDynamics::GyroscopeMeasurementDynamics() = default;

RDE::GyroscopeMeasurementDynamics::~GyroscopeMeasurementDynamics() = default;

bool RDE::GyroscopeMeasurementDynamics::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[GyroscopeMeasurementDynamics::initialize]";

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

    // Set the bias related variables if use_bias is true
    if (!ptr->getParameter("use_bias", m_useBias))
    {
        log()->info("{} Variable use_bias not found. Set to false by default.", errorPrefix);
    } else
    {
        m_biasVariableName = m_name + "_bias";
    }

    m_description = "Gyroscope measurement dynamics";

    m_isInitialized = true;

    return true;
}

bool RDE::GyroscopeMeasurementDynamics::finalize(
    const System::VariablesHandler& stateVariableHandler)
{
    constexpr auto errorPrefix = "[GyroscopeMeasurementDynamics::finalize]";

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
        if (m_subModelList[submodelIndex].hasGyroscope(m_name))
        {
            m_subModelWithGyro.push_back(submodelIndex);
        }
    }

    m_covariances.resize(m_covSingleVar.size() * m_subModelWithGyro.size());
    for (int index = 0; index < m_subModelWithGyro.size(); index++)
    {
        m_covariances.segment(index * m_covSingleVar.size(), m_covSingleVar.size())
            = m_covSingleVar;
    }

    m_size = m_covariances.size();

    m_jointVelocityFullModel.resize(m_stateVariableHandler.getVariable("ds").size);
    m_jointVelocityFullModel.setZero();

    m_subModelJointVel.resize(m_nrOfSubDynamics);

    for (int idx = 0; idx < m_nrOfSubDynamics; idx++)
    {
        m_subModelJointVel[idx].resize(m_subModelList[idx].getJointMapping().size());
        m_subModelJointVel[idx].setZero();
    }

    m_bias.resize(m_covSingleVar.size());
    m_bias.setZero();

    m_updatedVariable.resize(m_size);
    m_updatedVariable.setZero();

    return true;
}

bool RDE::GyroscopeMeasurementDynamics::setSubModels(
    const std::vector<SubModel>& subModelList,
    const std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList)
{
    constexpr auto errorPrefix = "[GyroscopeMeasurementDynamics::setSubModels]";

    m_subModelList = subModelList;
    m_subModelKinDynList = kinDynWrapperList;

    if (m_subModelList.size() == 0 || m_subModelKinDynList.size() == 0
        || m_subModelList.size() != m_subModelKinDynList.size())
    {
        log()->error("{} Wrong size of input parameters", errorPrefix);
        return false;
    }

    m_nrOfSubDynamics = m_subModelList.size();

    m_isSubModelListSet = true;

    return true;
}

bool RDE::GyroscopeMeasurementDynamics::checkStateVariableHandler()
{
    constexpr auto errorPrefix = "[GyroscopeMeasurementDynamics::checkStateVariableHandler]";

    if (!m_stateVariableHandler.getVariable("ds").isValid())
    {
        log()->error("{} The variable handler does not contain the expected state with name `ds`.",
                     errorPrefix);
        return false;
    }

    if (!m_useBias)
    {
        return true;
    }
    if (!m_stateVariableHandler.getVariable(m_biasVariableName).isValid())
    {
        log()->error("{} The variable handler does not contain the expected state with name `{}`.",
                     errorPrefix,
                     m_biasVariableName);
        return false;
    }

    return true;
}

bool RDE::GyroscopeMeasurementDynamics::update()
{
    for (int index = 0; index < m_subModelWithGyro.size(); index++)
    {
        m_accelerometerVelocity = Conversions::toManifTwist(
            m_subModelKinDynList[m_subModelWithGyro[index]]->getFrameVel(
                m_subModelList[m_subModelWithGyro[index]].getGyroscope(m_name).index));

        m_updatedVariable.segment(index * m_covSingleVar.size(), m_covSingleVar.size())
            = m_accelerometerVelocity.ang();

        if (m_useBias)
        {
            m_updatedVariable.segment(index * m_covSingleVar.size(), m_covSingleVar.size())
                += m_bias;
        }
    }

    return true;
}

void RDE::GyroscopeMeasurementDynamics::setState(const Eigen::Ref<const Eigen::VectorXd> ukfState)
{
    m_jointVelocityFullModel = ukfState.segment(m_stateVariableHandler.getVariable("ds").offset,
                                                m_stateVariableHandler.getVariable("ds").size);

    for (int smIndex = 0; smIndex < m_subModelList.size(); smIndex++)
    {
        for (int jntIndex = 0; jntIndex < m_subModelList[smIndex].getModel().getNrOfDOFs();
             jntIndex++)
        {
            m_subModelJointVel[smIndex][jntIndex]
                = m_jointVelocityFullModel[m_subModelList[smIndex].getJointMapping()[jntIndex]];
        }
    }

    if (m_useBias)
    {
        m_bias = ukfState.segment(m_stateVariableHandler.getVariable(m_biasVariableName).offset,
                                  m_stateVariableHandler.getVariable(m_biasVariableName).size);
    }
}

void RDE::GyroscopeMeasurementDynamics::setInput(const UKFInput& ukfInput)
{
    m_ukfInput = ukfInput;
}
