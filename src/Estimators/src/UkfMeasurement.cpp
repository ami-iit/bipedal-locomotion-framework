/**
 * @file UkfMeasurement.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <map>

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/Math/Constants.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelKinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/UkfMeasurement.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/ZeroVelocityDynamics.h>

namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

struct RDE::UkfMeasurement::Impl
{
    bool isInitialized{false};
    bool isFinalized{false};

    bfl::VectorDescription measurementDescription_;
    bfl::VectorDescription inputDescription_;

    Eigen::Vector3d gravity{0, 0, -Math::StandardAccelerationOfGravitation}; /**< Gravity vector. */

    Eigen::MatrixXd covarianceR; /**< Covariance matrix. */
    std::size_t measurementSize; /**< Length of the measurement vector. */
    double dT; /**< Sampling time */

    std::map<std::string, std::shared_ptr<Dynamics>> dynamicsList; /**< List of the dynamics composing the process model. */

    System::VariablesHandler measurementVariableHandler; /**< Variable handler describing the measurement vector. */
    System::VariablesHandler stateVariableHandler; /**< Variable handler describing the state vector. */

    std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel; /**< KinDynComputation object for the full model. */
    std::vector<SubModel> subModelList; /**< List of SubModel object describing the sub-models composing the full model. */
    std::vector<std::shared_ptr<SubModelKinDynWrapper>> kinDynWrapperList; /**< List of SubModelKinDynWrapper objects containing kinematic and dynamic information specific of each sub-model. */

    std::shared_ptr<const UkfInputProvider> ukfInputProvider; /**< Provider containing the updated robot state. */
    UKFInput ukfInput; /**< Struct containing the inputs for the ukf populated by the ukfInputProvider. */

    Eigen::VectorXd jointVelocityState; /**< Joint velocity compute by the ukf. */

    Eigen::VectorXd currentState; /**< State estimated in the previous step. */
    Eigen::VectorXd predictedMeasurement; /**< Vector containing the updated measurement. */

    Eigen::VectorXd measurement; /**< Measurements coming from the sensors. */

    std::map<std::string, Eigen::VectorXd> measurementMap; /**< Measurement map <measurement name, measurement value>. */

    int offsetMeasurement; /**< Offset used to fill the measurement vector. */
};

RDE::UkfMeasurement::UkfMeasurement()
{
    m_pimpl = std::make_unique<RDE::UkfMeasurement::Impl>();
}

RDE::UkfMeasurement::~UkfMeasurement() = default;

bool RDE::UkfMeasurement::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[UkfMeasurement::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("{} The parameter handler is not valid.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("sampling_time", m_pimpl->dT))
    {
        BipedalLocomotion::log()->error("{} Unable to find the `sampling_time` variable", logPrefix);
        return false;
    }

    m_pimpl->isInitialized = true;

    return true;
}

bool RDE::UkfMeasurement::finalize(const System::VariablesHandler& handler)
{
    constexpr auto logPrefix = "[UkfMeasurement::finalize]";

    if (!m_pimpl->isInitialized)
    {
        BipedalLocomotion::log()->error("{} Please call initialize() before finalize().", logPrefix);
        return false;
    }

    m_pimpl->isFinalized = false;

    m_pimpl->measurementSize = 0;

    // finalize all the dynamics
    for (auto& [name, dynamics] : m_pimpl->dynamicsList)
    {
        if(!dynamics->finalize(handler))
        {
            BipedalLocomotion::log()->error("{} Error while finalizing the dynamics named {}", logPrefix, name);
            return false;
        }

        m_pimpl->measurementSize += dynamics->size();
    }

    // Set value of measurement covariance matrix
    m_pimpl->covarianceR.resize(m_pimpl->measurementSize, m_pimpl->measurementSize);

    for (auto& [name, dynamics] : m_pimpl->dynamicsList)
    {
        m_pimpl->measurementVariableHandler.addVariable(name, dynamics->getCovariance().size());

        m_pimpl->covarianceR.block(m_pimpl->measurementVariableHandler.getVariable(name).offset, m_pimpl->measurementVariableHandler.getVariable(name).offset,
                                   m_pimpl->measurementVariableHandler.getVariable(name).size, m_pimpl->measurementVariableHandler.getVariable(name).size) = dynamics->getCovariance().asDiagonal();
    }

    m_pimpl->jointVelocityState.resize(m_pimpl->kinDynFullModel->model().getNrOfDOFs());

    m_pimpl->currentState.resize(m_pimpl->measurementSize);
    m_pimpl->currentState.setZero();

    m_pimpl->predictedMeasurement.resize(m_pimpl->measurementSize);
    m_pimpl->predictedMeasurement.setZero();

    m_pimpl->measurement.resize(m_pimpl->measurementSize);
    m_pimpl->measurement.setZero();

    m_pimpl->isFinalized = true;

    return true;
}

std::unique_ptr<RDE::UkfMeasurement> RDE::UkfMeasurement::build(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                                                                System::VariablesHandler& stateVariableHandler,
                                                                std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel,
                                                                const std::vector<SubModel>& subModelList,
                                                                const std::vector<std::shared_ptr<SubModelKinDynWrapper>>& kinDynWrapperList)
{
    constexpr auto logPrefix = "[UkfMeasurement::build]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("{} Invalid parameter handler.", logPrefix);
        return nullptr;
    }

    std::unique_ptr<RDE::UkfMeasurement> measurement = std::make_unique<RDE::UkfMeasurement>();

    if (!measurement->initialize(ptr))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the measurement object of the ukf.", logPrefix);
        return nullptr;
    }

    std::vector<std::string> dynamicsList;
    if (!ptr->getParameter("dynamics_list", dynamicsList))
    {
        BipedalLocomotion::log()->error("{} Unable to find the parameter 'dynamics_list'.", logPrefix);
        return nullptr;
    }

    if (kinDynFullModel == nullptr)
    {
        BipedalLocomotion::log()->error("{} The pointer to the `KinDynComputation` object is not valid.", logPrefix);
        return nullptr;
    }

    measurement->m_pimpl->stateVariableHandler = stateVariableHandler;

    // Set KinDynComputation for the full model
    measurement->m_pimpl->kinDynFullModel = kinDynFullModel;

    // Set the list of SubModel
    measurement->m_pimpl->subModelList.reserve(subModelList.size());
    measurement->m_pimpl->subModelList = subModelList;

    // set the list of SubModelKinDynWrapper
    measurement->m_pimpl->kinDynWrapperList.reserve(kinDynWrapperList.size());
    measurement->m_pimpl->kinDynWrapperList = kinDynWrapperList;

    // per each dynamics add variable to the variableHandler
    // and add the dynamics to the list
    for (const auto& dynamicsGroupName : dynamicsList)
    {
        auto dynamicsGroupTmp = ptr->getGroup(dynamicsGroupName).lock();
        if (dynamicsGroupTmp == nullptr)
        {
            BipedalLocomotion::log()->error("{} Unable to find the group '{}'.", logPrefix, dynamicsGroupName);
            return nullptr;
        }

        // add dT parameter since it is required by all the dynamics
        auto dynamicsGroup = dynamicsGroupTmp->clone();
        dynamicsGroup->setParameter("sampling_time", measurement->m_pimpl->dT);

        // create variable handler
        std::string dynamicsName;
        std::vector<double> covariances;
        std::vector<std::string> dynamicsElements;
        if (!dynamicsGroup->getParameter("name", dynamicsName))
        {
            BipedalLocomotion::log()->error("{} Unable to find the parameter 'name'.", logPrefix);
            return nullptr;
        }
        if (!dynamicsGroup->getParameter("covariance", covariances))
        {
            BipedalLocomotion::log()->error("{} Unable to find the parameter 'covariance'.", logPrefix);
            return nullptr;
        }
        if (!dynamicsGroup->getParameter("elements", dynamicsElements))
        {
            BipedalLocomotion::log()->error("{} Unable to find the parameter 'elements'.", logPrefix);
            return nullptr;
        }

        std::string dynamicModel;
        if (!dynamicsGroup->getParameter("dynamic_model", dynamicModel))
        {
            BipedalLocomotion::log()->error("{} Unable to find the parameter 'dynamic_model'.", logPrefix);
            return nullptr;
        }

        std::shared_ptr<Dynamics> dynamicsInstance = RDE::DynamicsFactory::createInstance(dynamicModel);
        if (dynamicsInstance == nullptr)
        {
            BipedalLocomotion::log()->error("{} The dynamic model '{}' has not been registered.", logPrefix, dynamicModel);
            return nullptr;
        }

        dynamicsInstance->setSubModels(subModelList, kinDynWrapperList);

        dynamicsInstance->initialize(dynamicsGroup);

        // add dynamics to the list
        measurement->m_pimpl->dynamicsList.insert({dynamicsName, dynamicsInstance});
    }

    // finalize estimator
    if (!measurement->finalize(measurement->m_pimpl->stateVariableHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to finalize the RobotDynamicsEstimator.", logPrefix);
        return nullptr;
    }

    return measurement;
}

void RDE::UkfMeasurement::setUkfInputProvider(std::shared_ptr<const UkfInputProvider> ukfInputProvider)
{
    m_pimpl->ukfInputProvider = ukfInputProvider;
}

std::pair<bool, Eigen::MatrixXd> RDE::UkfMeasurement::getNoiseCovarianceMatrix() const
{
    return std::make_pair(true, m_pimpl->covarianceR);
}

bfl::VectorDescription RDE::UkfMeasurement::getMeasurementDescription() const
{
    return bfl::VectorDescription(m_pimpl->measurementSize);
}

bfl::VectorDescription RDE::UkfMeasurement::getInputDescription() const
{
    return bfl::VectorDescription(m_pimpl->currentState.size(), 0, m_pimpl->measurementSize);
}

std::pair<bool, bfl::Data> RDE::UkfMeasurement::predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const
{
    constexpr auto logPrefix = "[UkfMeasurement::propagate]";

    // Check that everything is initialized and set
    if (!m_pimpl->isFinalized)
    {
        BipedalLocomotion::log()->error("{} The ukf state is not well initialized.", logPrefix);
        throw std::runtime_error("Error");
    }

    if (m_pimpl->ukfInputProvider == nullptr)
    {
        BipedalLocomotion::log()->error("{} The ukf input provider is not set.", logPrefix);
        throw std::runtime_error("Error");
    }

    // Get input of ukf from provider
    m_pimpl->ukfInput = m_pimpl->ukfInputProvider->getOutput();

    m_pimpl->currentState = cur_states;

    m_pimpl->jointVelocityState = m_pimpl->currentState.segment(m_pimpl->stateVariableHandler.getVariable("ds").offset,
                                                                m_pimpl->stateVariableHandler.getVariable("ds").size);

    // Update kindyn full model
    m_pimpl->kinDynFullModel->setRobotState(m_pimpl->ukfInput.robotBasePose.transform(),
                                            m_pimpl->ukfInput.robotJointPositions,
                                            iDynTree::make_span(m_pimpl->ukfInput.robotBaseVelocity.data(), manif::SE3d::Tangent::DoF),
                                            m_pimpl->jointVelocityState,
                                            m_pimpl->gravity);

    // Update kindyn sub-models
    for (int subModelIdx = 0; subModelIdx < m_pimpl->subModelList.size(); subModelIdx++)
    {
        m_pimpl->kinDynWrapperList[subModelIdx]->updateInternalKinDynState();
    }

    // TODO
    // This could be parallelized

    // Update all the dynamics
    for (auto& [name, dynamics] : m_pimpl->dynamicsList)
    {
        dynamics->setState(m_pimpl->currentState);

        dynamics->setInput(m_pimpl->ukfInput);

        if (!dynamics->update())
        {
            BipedalLocomotion::log()->error("{} Cannot update the dynamics with name `{}`.", logPrefix, name);
            throw std::runtime_error("Error");
        }

        m_pimpl->predictedMeasurement.segment(m_pimpl->measurementVariableHandler.getVariable(name).offset,
                                              m_pimpl->measurementVariableHandler.getVariable(name).size) = dynamics->getUpdatedVariable();
    }

    return std::make_pair(true, std::move(m_pimpl->predictedMeasurement));
}

std::pair<bool, bfl::Data> RDE::UkfMeasurement::innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const
{
    Eigen::MatrixXd innovation = -(bfl::any::any_cast<Eigen::MatrixXd>(predicted_measurements).colwise()
                                   - bfl::any::any_cast<Eigen::MatrixXd>(measurements).col(0));

    return std::make_pair(true, std::move(innovation));
}

std::size_t RDE::UkfMeasurement::getMeasurementSize()
{
    return m_pimpl->measurementSize;
}

BipedalLocomotion::System::VariablesHandler RDE::UkfMeasurement::getMeasurementVariableHandler()
{
    return m_pimpl->measurementVariableHandler;
}

void RDE::UkfMeasurement::setStateVariableHandler(System::VariablesHandler stateVariableHandler) const
{
    m_pimpl->stateVariableHandler = stateVariableHandler;
}

std::pair<bool, bfl::Data> RDE::UkfMeasurement::measure(const bfl::Data& data) const
{
    return std::make_pair(true, m_pimpl->measurement);
}

bool RDE::UkfMeasurement::freeze(const bfl::Data& data)
{
    m_pimpl->measurementMap = bfl::any::any_cast<std::map<std::string, Eigen::VectorXd>>(data);

    for (auto& [name, dynamics] : m_pimpl->dynamicsList)
    {
        m_pimpl->offsetMeasurement = m_pimpl->measurementVariableHandler.getVariable(name).offset;

        while(m_pimpl->offsetMeasurement < (m_pimpl->measurementVariableHandler.getVariable(name).offset + m_pimpl->measurementVariableHandler.getVariable(name).size))
        {
            m_pimpl->measurement.segment(m_pimpl->offsetMeasurement, m_pimpl->measurementMap[name].size())
                    = m_pimpl->measurementMap[name];

            m_pimpl->offsetMeasurement += m_pimpl->measurementMap[name].size();
        }
    }

    return true;
}
