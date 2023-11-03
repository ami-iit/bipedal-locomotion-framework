/**
 * @file RobotDynamicsEstimator.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BayesFilters/UKFPrediction.h>
#include <BayesFilters/UKFCorrection.h>
#include <BayesFilters/Gaussian.h>

#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/KinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/UkfState.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/UkfMeasurement.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/RobotDynamicsEstimator.h>

using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion;

struct RobotDynamicsEstimator::Impl
{
    RobotDynamicsEstimatorOutput estimatorOutput; /**< Output of the estimator. */

    UKFInput ukfInput; /**< Object to set the provider. */
    std::shared_ptr<UkfInputProvider> inputProvider; /**< Shared pointer used by all the dynamics. It needs to be updated here. */

    std::map<std::string, Eigen::VectorXd> ukfMeasurementFromSensors; /**< This map containes the measurement coming from the sensors
                                                                        and needed for the correction phase of the estimation. */

    bfl::Gaussian predictedState; /**< Predicted state computed by the `predict` method. */
    bfl::Gaussian correctedState; /**< Corrected state computed by the `correct` method. */

    double dT; /**< Sampling time. */
    double alpha, beta, kappa; /**< Parameters of the unscented kalman filter. */

    std::unique_ptr<UkfState> stateModel; /**< UKF state model. */
    std::unique_ptr<UkfMeasurement> measurementModel; /**< UKF measurement model. */

    std::unique_ptr<bfl::UKFPrediction> ukfPrediction;
    std::unique_ptr<bfl::UKFCorrection> ukfCorrection;

    System::VariablesHandler stateHandler; /**< Handler of the ukf state. */
    System::VariablesHandler measurementHandler; /**< Handler of the ukf measurement. */

    Eigen::MatrixXd initialStateCovariance;

    bool isInitialized{false};
    bool isFinalized{false};

    bool isValid{false};

    bool isInitialStateSet{false};
};

RobotDynamicsEstimator::RobotDynamicsEstimator()
{
    m_pimpl = std::make_unique<RobotDynamicsEstimator::Impl>();
}

RobotDynamicsEstimator::~RobotDynamicsEstimator() = default;

bool RobotDynamicsEstimator::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[RobotDynamicsEstimator::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", logPrefix);
        return false;
    }

    auto groupGeneral = ptr->getGroup("GENERAL").lock();
    if (groupGeneral == nullptr)
    {
        log()->error("{} Error while retrieving the group GENERAL.", logPrefix);
        return false;
    }

    if (!groupGeneral->getParameter("sampling_time", m_pimpl->dT))
    {
        log()->error("{} Error while retrieving the sampling time variable.", logPrefix);
        return false;
    }

    auto groupUkf = ptr->getGroup("UKF").lock();
    if (groupUkf == nullptr)
    {
        log()->error("{} Error while retrieving the group UKF.", logPrefix);
        return false;
    }

    if (!groupUkf->getParameter("alpha", m_pimpl->alpha))
    {
        log()->error("{} Error while retrieving the alpha variable.", logPrefix);
        return false;
    }

    if (!groupUkf->getParameter("beta", m_pimpl->beta))
    {
        log()->error("{} Error while retrieving the beta variable.", logPrefix);
        return false;
    }

    if (!groupUkf->getParameter("kappa", m_pimpl->kappa))
    {
        log()->error("{} Error while retrieving the kappa variable.", logPrefix);
        return false;
    }

    m_pimpl->inputProvider = std::make_shared<UkfInputProvider>();

    m_pimpl->isInitialized = true;

    return true;
}

bool RobotDynamicsEstimator::finalize(const System::VariablesHandler& stateVariableHandler,
                                      const System::VariablesHandler& measurementVariableHandler,
                                      std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel)
{
    // Resize variables for the estimation
    m_pimpl->predictedState.resize(stateVariableHandler.getNumberOfVariables());
    m_pimpl->predictedState.mean().setZero();
    m_pimpl->predictedState.covariance() = m_pimpl->initialStateCovariance;

    m_pimpl->correctedState.resize(stateVariableHandler.getNumberOfVariables());
    m_pimpl->correctedState.mean().setZero();
    m_pimpl->correctedState.covariance() = m_pimpl->initialStateCovariance;

    // Set the provider initial state
    m_pimpl->ukfInput.robotBasePose.setIdentity();
    m_pimpl->ukfInput.robotBaseVelocity.setZero();
    m_pimpl->ukfInput.robotBaseAcceleration.setZero();
    m_pimpl->ukfInput.robotJointPositions.resize(kinDynFullModel->model().getNrOfDOFs());
    m_pimpl->ukfInput.robotJointAccelerations.resize(kinDynFullModel->model().getNrOfDOFs());
    m_pimpl->inputProvider->setInput(m_pimpl->ukfInput);

    m_pimpl->isFinalized = true;

    return m_pimpl->isFinalized;
}

std::unique_ptr<RobotDynamicsEstimator> RobotDynamicsEstimator::build(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                                                                      std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel,
                                                                      const std::vector<SubModel>& subModelList,
                                                                      const std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList)
{
    constexpr auto logPrefix = "[RobotDynamicsEstimator::build]";

    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return nullptr;
    }

    if (!kinDynFullModel->setFrameVelocityRepresentation(iDynTree::BODY_FIXED_REPRESENTATION))
    {
        log()->error("{} Cannot set the frame velocity representation on the `iDynTree::KinDynComputation` "
                     "object describing the full robot model.", logPrefix);
        return nullptr;
    }

    std::unique_ptr<RobotDynamicsEstimator> estimator = std::make_unique<RobotDynamicsEstimator>();

    if (!estimator->initialize(handler))
    {
        log()->error("{} Error initializing the RobotDynamicsEstimator.", logPrefix);
        return nullptr;
    }

    auto groupUKF = ptr->getGroup("UKF").lock();
    if (groupUKF == nullptr)
    {
        log()->error("{} Error while retrieving the group UKF.", logPrefix);
        return nullptr;
    }

    auto groupUKFStateTmp = groupUKF->getGroup("UKF_STATE").lock();
    if (groupUKFStateTmp == nullptr)
    {
        log()->error("{} Error while retrieving the group UKF_STATE.", logPrefix);
        return nullptr;
    }

    auto groupUKFState = groupUKFStateTmp->clone();
    groupUKFState->setParameter("sampling_time", estimator->m_pimpl->dT);

    // Step 1
    // Create the state model for the ukf
    estimator->m_pimpl->stateModel = UkfState::build(groupUKFState,
                                                     kinDynFullModel,
                                                     subModelList,
                                                     kinDynWrapperList);
    if (estimator->m_pimpl->stateModel == nullptr)
    {
        log()->error("{} Error while creating the ukf state model.", logPrefix);
        return nullptr;
    }

    // Save a copy of the state variable handler
    estimator->m_pimpl->stateHandler = estimator->m_pimpl->stateModel->getStateVariableHandler();

    // Set the input provider
    estimator->m_pimpl->stateModel->setUkfInputProvider(estimator->m_pimpl->inputProvider);

    // Get initial state covariance
    estimator->m_pimpl->initialStateCovariance = estimator->m_pimpl->stateModel->getInitialStateCovarianceMatrix();

    // Step 2
    // Initialize the unscented Kalman filter prediction step and pass the ownership of the state model
    std::unique_ptr<bfl::AdditiveStateModel> stateModelTemp = std::move(estimator->m_pimpl->stateModel);
    estimator->m_pimpl->ukfPrediction = std::make_unique<bfl::UKFPrediction>(std::move(stateModelTemp),
                                                                             estimator->m_pimpl->alpha,
                                                                             estimator->m_pimpl->beta,
                                                                             estimator->m_pimpl->kappa);

    // Step 3
    // Create the measurement model for the ukf
    auto groupUKFMeasTmp = groupUKF->getGroup("UKF_MEASUREMENT").lock();
    if (groupUKFMeasTmp == nullptr)
    {
        log()->error("{} Error while retrieving the group UKF_MEASUREMENT.", logPrefix);
        return nullptr;
    }

    auto groupUKFMeas = groupUKFMeasTmp->clone();
    groupUKFMeas->setParameter("sampling_time", estimator->m_pimpl->dT);

    // Create the measurement model for the ukf
    estimator->m_pimpl->measurementModel = UkfMeasurement::build(groupUKFMeas,
                                                                 estimator->m_pimpl->stateHandler,
                                                                 kinDynFullModel,
                                                                 subModelList,
                                                                 kinDynWrapperList);
    if (estimator->m_pimpl->measurementModel == nullptr)
    {
        log()->error("{} Error while creating the ukf measurement model.", logPrefix);
        return nullptr;
    }

    // Save a copy of the measurement variable handler
    estimator->m_pimpl->measurementHandler = estimator->m_pimpl->measurementModel->getMeasurementVariableHandler();

    // Set the input provider
    estimator->m_pimpl->measurementModel->setUkfInputProvider(estimator->m_pimpl->inputProvider);

    // Step 4
    // Initialize the unscented Kalman filter correction step and pass the ownership of the measurement model.
    std::unique_ptr<bfl::AdditiveMeasurementModel> measurementModelTemp = std::move(estimator->m_pimpl->measurementModel);
    estimator->m_pimpl->ukfCorrection = std::make_unique<bfl::UKFCorrection>(std::move(measurementModelTemp),
                                                                             estimator->m_pimpl->alpha,
                                                                             estimator->m_pimpl->beta,
                                                                             estimator->m_pimpl->kappa);

    // Finalize the estimator
    estimator->finalize(estimator->m_pimpl->stateHandler, estimator->m_pimpl->measurementHandler, kinDynFullModel);

    return estimator;
}

bool RobotDynamicsEstimator::setInitialState(const RobotDynamicsEstimatorOutput& initialState)
{
    constexpr auto logPrefix = "[RobotDynamicsEstimator::setInitialState]";

    System::VariablesHandler::VariableDescription variable;

    if (m_pimpl->stateHandler.getVariable("ds", variable))
    {
        if (initialState.ds.size() != variable.size)
        {
            log()->error("{} Wrong size of variable `ds`. Found {}, expected {}.", logPrefix, initialState.ds.size(), variable.size);
            return false;
        }
        m_pimpl->correctedState.mean().segment(variable.offset, variable.size) = initialState.ds;
    }

    if (m_pimpl->stateHandler.getVariable("tau_m", variable))
    {
        if (initialState.tau_m.size() != variable.size)
        {
            log()->error("{} Wrong size of variable `tau_m`. Found {}, expected {}.", logPrefix, initialState.tau_m.size(), variable.size);
            return false;
        }
        m_pimpl->correctedState.mean().segment(variable.offset, variable.size) = initialState.tau_m;
    }

    if (m_pimpl->stateHandler.getVariable("tau_F", variable))
    {
        if (initialState.tau_F.size() != variable.size)
        {
            log()->error("{} Wrong size of variable `tau_F`. Found {}, expected {}.", logPrefix, initialState.tau_F.size(), variable.size);
            return false;
        }
        m_pimpl->correctedState.mean().segment(variable.offset, variable.size) = initialState.tau_F;
    }

    for (auto const& [key, val] : initialState.ftWrenches)
    {
        if (m_pimpl->stateHandler.getVariable(key, variable))
        {
            if (val.size() != variable.size)
            {
                log()->error("{} Wrong size of variable `{}`. Found {}, expected {}.", logPrefix, val, val.size(), variable.size);
                return false;
            }
            m_pimpl->correctedState.mean().segment(variable.offset, variable.size) = val;
        }
    }

    for (auto const& [key, val] : initialState.ftWrenchesBiases)
    {
        if (m_pimpl->stateHandler.getVariable(key, variable))
        {
            if (val.size() != variable.size)
            {
                log()->error("{} Wrong size of variable `{}`. Found {}, expected {}.", logPrefix, val, val.size(), variable.size);
                return false;
            }
            m_pimpl->correctedState.mean().segment(variable.offset, variable.size) = val;
        }
    }

    for (auto const& [key, val] : initialState.gyroscopeBiases)
    {
        if (m_pimpl->stateHandler.getVariable(key, variable))
        {
            if (val.size() != variable.size)
            {
                log()->error("{} Wrong size of variable `{}`. Found {}, expected {}.", logPrefix, val, val.size(), variable.size);
                return false;
            }
            m_pimpl->correctedState.mean().segment(variable.offset, variable.size) = val;
        }
    }

    for (auto const& [key, val] : initialState.accelerometerBiases)
    {
        if (m_pimpl->stateHandler.getVariable(key, variable))
        {
            if (val.size() != variable.size)
            {
                log()->error("{} Wrong size of variable `{}`. Found {}, expected {}.", logPrefix, val, val.size(), variable.size);
                return false;
            }
            m_pimpl->correctedState.mean().segment(variable.offset, variable.size) = val;
        }
    }

    for (auto const& [key, val] : initialState.contactWrenches)
    {
        if (m_pimpl->stateHandler.getVariable(key, variable))
        {
            if (val.size() != variable.size)
            {
                log()->error("{} Wrong size of variable `{}`. Found {}, expected {}.", logPrefix, val, val.size(), variable.size);
                return false;
            }
            m_pimpl->correctedState.mean().segment(variable.offset, variable.size) = val;
        }
    }

    m_pimpl->estimatorOutput = initialState;

    m_pimpl->isInitialStateSet = true;

    return m_pimpl->isInitialStateSet;
}

bool RobotDynamicsEstimator::isOutputValid() const
{
    return m_pimpl->isValid;
}

bool RobotDynamicsEstimator::advance()
{
    constexpr auto logPrefix = "[RobotDynamicsEstimator::advance]";

    // when advance is called the previous solution is no more valid
    m_pimpl->isValid = false;

    if (!m_pimpl->isInitialized)
    {
        log()->error("{} Please call initialize() before advance().", logPrefix);
        return false;
    }

    if (!m_pimpl->isFinalized)
    {
        log()->error("{} Please call finalize() before advance().", logPrefix);
        return false;
    }

    if (!m_pimpl->isInitialStateSet)
    {
        log()->error("{} Please set the initial state before advance().", logPrefix);
        return false;
    }

    // Step 1 --> Predict
    m_pimpl->ukfPrediction->predict(m_pimpl->correctedState, m_pimpl->predictedState);

    // Step 2 --> Set measurement
    if (!m_pimpl->ukfCorrection->freeze_measurements(m_pimpl->ukfMeasurementFromSensors))
    {
        log()->error("{} Cannot set the measurement to the UkfCorrection object.", logPrefix);
        return false;
    }

    // Step 3 --> Correct
    m_pimpl->ukfCorrection->correct(m_pimpl->predictedState, m_pimpl->correctedState);

    m_pimpl->isValid = true;

    return true;
}

bool RobotDynamicsEstimator::setInput(const RobotDynamicsEstimatorInput & input)
{
    constexpr auto logPrefix = "[RobotDynamicsEstimator::setInput]";

    // Set the input provider state
    m_pimpl->ukfInput.robotBasePose = input.basePose;
    m_pimpl->ukfInput.robotBaseVelocity = input.baseVelocity;
    m_pimpl->ukfInput.robotBaseAcceleration = input.baseAcceleration;
    m_pimpl->ukfInput.robotJointPositions = input.jointPositions;
    m_pimpl->ukfInput.robotJointAccelerations.setZero();

    if (!m_pimpl->inputProvider->setInput(m_pimpl->ukfInput))
    {
        log()->error("{} Cannot set the input of the input provider.", logPrefix);
        return false;
    }

    // Set the `std::map<std::string, Eigen::VectorXd>` used as measurement object
    // for the freeze method of the UkfCorrection
    m_pimpl->ukfMeasurementFromSensors["ds"] = input.jointVelocities;
    m_pimpl->ukfMeasurementFromSensors["i_m"] = input.motorCurrents;
    m_pimpl->ukfMeasurementFromSensors["dv_base"] = input.baseAcceleration.coeffs();
    for (auto & [key, value] : input.ftWrenches)
    {
        m_pimpl->ukfMeasurementFromSensors[key] = value;
    }
    for (auto & [key, value] : input.linearAccelerations)
    {
        m_pimpl->ukfMeasurementFromSensors[key] = value;
    }
    for (auto & [key, value] : input.angularVelocities)
    {
        m_pimpl->ukfMeasurementFromSensors[key] = value;
    }
    m_pimpl->ukfMeasurementFromSensors["tau_F"] = input.friction;

    return true;
}

const RobotDynamicsEstimatorOutput& RobotDynamicsEstimator::getOutput() const
{
     constexpr auto logPrefix = "[RobotDynamicsEstimator::getOutput]";

    if (m_pimpl->isValid)
    {
        m_pimpl->estimatorOutput.ds = m_pimpl->correctedState.mean().segment(m_pimpl->stateHandler.getVariable("ds").offset,
                                                                             m_pimpl->stateHandler.getVariable("ds").size);

        m_pimpl->estimatorOutput.tau_m = m_pimpl->correctedState.mean().segment(m_pimpl->stateHandler.getVariable("tau_m").offset,
                                                                                m_pimpl->stateHandler.getVariable("tau_m").size);

        m_pimpl->estimatorOutput.tau_F = m_pimpl->correctedState.mean().segment(m_pimpl->stateHandler.getVariable("tau_F").offset,
                                                                                m_pimpl->stateHandler.getVariable("tau_F").size);

        for (auto & [key, value] : m_pimpl->estimatorOutput.ftWrenches)
        {
            if (m_pimpl->stateHandler.getVariable(key).size > 0)
            {
                m_pimpl->estimatorOutput.ftWrenches[key] = m_pimpl->correctedState.mean().segment(m_pimpl->stateHandler.getVariable(key).offset,
                                                                                                  m_pimpl->stateHandler.getVariable(key).size);
            }
            else
            {
                log()->debug("{} Variable {} not found in the state vector.", logPrefix, key);
            }
        }

        for (auto & [key, value] : m_pimpl->estimatorOutput.ftWrenchesBiases)
        {
            if (m_pimpl->stateHandler.getVariable(key).size > 0)
            {
                m_pimpl->estimatorOutput.ftWrenchesBiases[key] = m_pimpl->correctedState.mean().segment(m_pimpl->stateHandler.getVariable(key).offset,
                                                                                                        m_pimpl->stateHandler.getVariable(key).size);
            }
            else
            {
                log()->debug("{} Variable {} not found in the state vector.", logPrefix, key);
            }
        }

        for (auto & [key, value] : m_pimpl->estimatorOutput.accelerometerBiases)
        {
            if (m_pimpl->stateHandler.getVariable(key).size > 0)
            {
                m_pimpl->estimatorOutput.accelerometerBiases[key] = m_pimpl->correctedState.mean().segment(m_pimpl->stateHandler.getVariable(key).offset,
                                                                                                           m_pimpl->stateHandler.getVariable(key).size);
            }
            else
            {
                log()->debug("{} Variable {} not found in the state vector.", logPrefix, key);
            }
        }

        for (auto & [key, value] : m_pimpl->estimatorOutput.gyroscopeBiases)
        {
            if (m_pimpl->stateHandler.getVariable(key).size > 0)
            {
                m_pimpl->estimatorOutput.gyroscopeBiases[key] = m_pimpl->correctedState.mean().segment(m_pimpl->stateHandler.getVariable(key).offset,
                                                                                                       m_pimpl->stateHandler.getVariable(key).size);
            }
            else
            {
                log()->debug("{} Variable {} not found in the state vector.", logPrefix, key);
            }
        }

        for (auto & [key, value] : m_pimpl->estimatorOutput.contactWrenches)
        {
            if (m_pimpl->stateHandler.getVariable(key).size > 0)
            {
                m_pimpl->estimatorOutput.contactWrenches[key] = m_pimpl->correctedState.mean().segment(m_pimpl->stateHandler.getVariable(key).offset,
                                                                                                       m_pimpl->stateHandler.getVariable(key).size);
            }
            else
            {
                log()->debug("{} Variable {} not found in the state vector.", logPrefix, key);
            }
        }
    }

    return m_pimpl->estimatorOutput;
}
