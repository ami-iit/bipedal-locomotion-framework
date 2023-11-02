/**
 * @file AccelerometerMeasurementDynamics.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <Eigen/Dense>

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/AccelerometerMeasurementDynamics.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

RDE::AccelerometerMeasurementDynamics::AccelerometerMeasurementDynamics() = default;

RDE::AccelerometerMeasurementDynamics::~AccelerometerMeasurementDynamics() = default;

bool RDE::AccelerometerMeasurementDynamics::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
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

    // Set the bias related variables if use_bias is true
    if (!ptr->getParameter("use_bias", m_useBias))
    {
        log()->info("{} Variable use_bias not found. Set to false by default.", errorPrefix);
    } else
    {
        m_biasVariableName = m_name + "_bias";
    }

    m_gravity.setZero();
    m_gravity[2] = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    m_JdotNu.resize(6);
    m_Jvdot.resize(6);
    m_Jsdotdot.resize(6);

    m_description = "Accelerometer measurement dynamics";

    m_isInitialized = true;

    return true;
}

bool RDE::AccelerometerMeasurementDynamics::finalize(
    const System::VariablesHandler& stateVariableHandler)
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
            m_accelerometerFrameName = m_subModelList[submodelIndex].getAccelerometer(m_name).frame;
        }
    }

    m_covariances.resize(m_covSingleVar.size() * m_subModelsWithAccelerometer.size());
    for (int index = 0; index < m_subModelsWithAccelerometer.size(); index++)
    {
        m_covariances.segment(index * m_covSingleVar.size(), m_covSingleVar.size())
            = m_covSingleVar;
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

    return true;
}

bool RDE::AccelerometerMeasurementDynamics::setSubModels(
    const std::vector<SubModel>& subModelList,
    const std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList)
{
    m_subModelList = subModelList;
    m_kinDynWrapperList = kinDynWrapperList;
    m_isSubModelListSet = true;

    return true;
}

bool RDE::AccelerometerMeasurementDynamics::checkStateVariableHandler()
{
    constexpr auto errorPrefix = "[AccelerometerMeasurementDynamics::checkStateVariableHandler]";

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

bool RDE::AccelerometerMeasurementDynamics::update()
{
    constexpr auto errorPrefix = "[AccelerometerMeasurementDynamics::update]";

    // compute joint acceleration per each sub-model containing the accelerometer
    for (int subDynamicsIdx = 0; subDynamicsIdx < m_subModelList.size(); subDynamicsIdx++)
    {
        if (m_subModelList[subDynamicsIdx].getModel().getNrOfDOFs() > 0)
        {
            for (int jointIdx = 0;
                 jointIdx < m_subModelList[subDynamicsIdx].getJointMapping().size();
                 jointIdx++)
            {
                m_subModelJointAcc[subDynamicsIdx][jointIdx]
                    = m_ukfInput.robotJointAccelerations[m_subModelList[subDynamicsIdx]
                                                             .getJointMapping()[jointIdx]];
            }
        }
    }

    for (int index = 0; index < m_subModelsWithAccelerometer.size(); index++)
    {
        m_subModelBaseAcceleration
            = m_kinDynWrapperList[m_subModelsWithAccelerometer[index]]->getNuDot().head(6);

        if (!m_kinDynWrapperList[m_subModelsWithAccelerometer[index]]
                 ->getFrameAcc(m_accelerometerFrameName,
                               iDynTree::make_span(m_subModelBaseAcceleration.data(),
                                                   manif::SE3d::Tangent::DoF),
                               m_subModelJointAcc[m_subModelsWithAccelerometer[index]],
                               iDynTree::make_span(m_accelerometerFameAcceleration.data(),
                                                   manif::SE3d::Tangent::DoF)))
        {
            log()->error("{} Failed while getting the accelerometer frame acceleration.",
                         errorPrefix);
            return false;
        }

        m_imuRworld
            = Conversions::toManifRot(m_kinDynWrapperList[m_subModelsWithAccelerometer[index]]
                                          ->getWorldTransform(m_accelerometerFrameName)
                                          .getRotation()
                                          .inverse());

        m_accRg = m_imuRworld.act(m_gravity);

        m_accelerometerFameVelocity = Conversions::toManifTwist(
            m_kinDynWrapperList[m_subModelsWithAccelerometer[index]]->getFrameVel(
                m_accelerometerFrameName));

        m_vCrossW = m_accelerometerFameVelocity.lin().cross(m_accelerometerFameVelocity.ang());

        m_updatedVariable.segment(index * m_covSingleVar.size(), m_covSingleVar.size())
            = m_accelerometerFameAcceleration.lin() - m_vCrossW - m_accRg;

        if (m_useBias)
        {
            m_updatedVariable.segment(index * m_covSingleVar.size(), m_covSingleVar.size())
                += m_bias;
        }
    }

    return true;
}

void RDE::AccelerometerMeasurementDynamics::setState(
    const Eigen::Ref<const Eigen::VectorXd> ukfState)
{
    if (m_useBias)
    {
        m_bias = ukfState.segment(m_stateVariableHandler.getVariable(m_biasVariableName).offset,
                                  m_stateVariableHandler.getVariable(m_biasVariableName).size);
    }
}

void RDE::AccelerometerMeasurementDynamics::setInput(const UKFInput& ukfInput)
{
    m_ukfInput = ukfInput;
}
