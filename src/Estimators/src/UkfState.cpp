/**
 * @file UkfState.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <map>

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/Math/Constants.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelKinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/UkfState.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/ZeroVelocityDynamics.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/FrictionTorqueStateDynamics.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/JointVelocityStateDynamics.h>

using namespace BipedalLocomotion;
namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

struct RDE::UkfState::Impl
{
    bool isInitialized{false};
    bool isFinalized{false};

    Eigen::Vector3d gravity{0, 0, -Math::StandardAccelerationOfGravitation}; /**< Gravity vector. */

    Eigen::MatrixXd covarianceQ; /**< Covariance matrix. */
    std::size_t stateSize; /**< Length of the state vector. */
    double dT; /**< Sampling time */

    std::map<std::string, std::shared_ptr<Dynamics>> dynamicsList; /**< List of the dynamics composing the process model. */

    System::VariablesHandler stateVariableHandler; /**< Variable handler describing the state vector. */

    std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel; /**< KinDynComputation object for the full model. */
    std::vector<SubModel> subModelList; /**< List of SubModel object describing the sub-models composing the full model. */
    std::vector<std::shared_ptr<SubModelKinDynWrapper>> kinDynWrapperList; /**< List of SubModelKinDynWrapper objects containing kinematic and dynamic information specific of each sub-model. */

    std::shared_ptr<const UkfInputProvider> ukfInputProvider; /**< Provider containing the updated robot state. */
    UKFInput ukfInput; /**< Struct containing the inputs for the ukf populated by the ukfInputProvider. */

    Eigen::VectorXd jointVelocityState; /**< Joint velocity compute by the ukf. */
    Eigen::VectorXd currentState; /**< State estimated in the previous step. */
    Eigen::VectorXd nextState; /**< Vector containing all the updated states. */
};

RDE::UkfState::UkfState()
{
    m_pimpl = std::make_unique<RDE::UkfState::Impl>();
}

RDE::UkfState::~UkfState() = default;

bool RDE::UkfState::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[UkfState::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("sampling_time", m_pimpl->dT))
    {
        log()->error("{} Unable to find the `sampling_time` variable", logPrefix);
        return false;
    }

    m_pimpl->isInitialized = true;

    return true;
}

bool RDE::UkfState::finalize(const System::VariablesHandler& handler)
{
    constexpr auto logPrefix = "[UkfState::finalize]";

    if (!m_pimpl->isInitialized)
    {
        log()->error("{} Please call initialize() before finalize().", logPrefix);
        return false;
    }

    m_pimpl->isFinalized = false;

    m_pimpl->stateSize = 0;

    // finalize all the dynamics
    for (auto& [name, dynamics] : m_pimpl->dynamicsList)
    {
        if(!dynamics->finalize(handler))
        {
            log()->error("{} Error while finalizing the dynamics names {}", logPrefix, name);
            return false;
        }

        m_pimpl->stateSize += dynamics->size();
    }


    // Set value of process covariance matrix
    m_pimpl->covarianceQ.resize(m_pimpl->stateSize, m_pimpl->stateSize);
    m_pimpl->covarianceQ.setZero();

    for (auto& [name, dynamics] : m_pimpl->dynamicsList)
    {
        m_pimpl->covarianceQ.block(handler.getVariable(name).offset, handler.getVariable(name).offset,
                                   handler.getVariable(name).size, handler.getVariable(name).size) = dynamics->getCovariance().asDiagonal();
    }

    m_pimpl->jointVelocityState.resize(m_pimpl->kinDynFullModel->model().getNrOfDOFs());

    m_pimpl->currentState.resize(m_pimpl->stateSize);
    m_pimpl->currentState.setZero();

    m_pimpl->nextState.resize(m_pimpl->stateSize);
    m_pimpl->nextState.setZero();

    m_pimpl->isFinalized = true;

    return true;
}

std::unique_ptr<RDE::UkfState> RDE::UkfState::build(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                                                    std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel,
                                                    const std::vector<SubModel>& subModelList,
                                                    const std::vector<std::shared_ptr<SubModelKinDynWrapper>>& kinDynWrapperList)
{
    constexpr auto logPrefix = "[UkfState::build]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return nullptr;
    }

    std::unique_ptr<RDE::UkfState> state = std::make_unique<RDE::UkfState>();

    if (!state->initialize(ptr))
    {
        log()->error("{} Unable to initialize the RobotDynamicsEstimator.", logPrefix);
        return nullptr;
    }

    std::vector<std::string> dynamicsList;
    if (!ptr->getParameter("dynamics_list", dynamicsList))
    {
        log()->error("{} Unable to find the parameter 'dynamics_list'.", logPrefix);
        return nullptr;
    }

    if (kinDynFullModel == nullptr)
    {
        log()->error("{} The pointer to the `KinDynComputation` object is not valid.", logPrefix);
        return nullptr;
    }

    // Set KinDynComputation for the full model
    state->m_pimpl->kinDynFullModel = kinDynFullModel;

    // Set the list of SubModel
    state->m_pimpl->subModelList.reserve(subModelList.size());
    state->m_pimpl->subModelList = subModelList;

    // set the list of SubModelKinDynWrapper
    state->m_pimpl->kinDynWrapperList.reserve(kinDynWrapperList.size());
    state->m_pimpl->kinDynWrapperList = kinDynWrapperList;

    // per each dynamics add variable to the variableHandler
    // and add the dynamics to the list
    for (const auto& dynamicsGroupName : dynamicsList)
    {
        auto dynamicsGroupTmp = ptr->getGroup(dynamicsGroupName).lock();
        if (dynamicsGroupTmp == nullptr)
        {
            log()->error("{} Unable to find the group '{}'.", logPrefix, dynamicsGroupName);
            return nullptr;
        }

        // add dT parameter since it is required by all the dynamics
        auto dynamicsGroup = dynamicsGroupTmp->clone();
        dynamicsGroup->setParameter("sampling_time", state->m_pimpl->dT);

        // create variable handler
        std::string dynamicsName;
        std::vector<double> covariances;
        std::vector<std::string> dynamicsElements;
        if (!dynamicsGroup->getParameter("name", dynamicsName))
        {
            log()->error("{} Unable to find the parameter 'name'.", logPrefix);
            return nullptr;
        }
        if (!dynamicsGroup->getParameter("covariance", covariances))
        {
            log()->error("{} Unable to find the parameter 'covariance'.", logPrefix);
            return nullptr;
        }
        if (!dynamicsGroup->getParameter("elements", dynamicsElements))
        {
            log()->error("{} Unable to find the parameter 'elements'.", logPrefix);
            return nullptr;
        }
        state->m_pimpl->stateVariableHandler.addVariable(dynamicsName, covariances.size(), dynamicsElements);

        std::string dynamicModel;
        if (!dynamicsGroup->getParameter("dynamic_model", dynamicModel))
        {
            log()->error("{} Unable to find the parameter 'dynamic_model'.", logPrefix);
            return nullptr;
        }

        std::shared_ptr<Dynamics> dynamicsInstance = RDE::DynamicsFactory::createInstance(dynamicModel);
        if (dynamicsInstance == nullptr)
        {
            log()->error("{} The dynamic model '{}' has not been registered.", logPrefix, dynamicModel);
            return nullptr;
        }

        dynamicsInstance->setSubModels(subModelList, kinDynWrapperList);

        dynamicsInstance->initialize(dynamicsGroup);

        // add dynamics to the list
        state->m_pimpl->dynamicsList.insert({dynamicsName, dynamicsInstance});
    }

    // finalize estimator
    if (!state->finalize(state->m_pimpl->stateVariableHandler))
    {
        log()->error("{} Unable to finalize the RobotDynamicsEstimator.", logPrefix);
        return nullptr;
    }

    return state;
}

void RDE::UkfState::setUkfInputProvider(std::shared_ptr<const UkfInputProvider> ukfInputProvider)
{
    m_pimpl->ukfInputProvider = ukfInputProvider;
}

Eigen::MatrixXd RDE::UkfState::getNoiseCovarianceMatrix()
{
    return m_pimpl->covarianceQ;
}

System::VariablesHandler RDE::UkfState::getStateVariableHandler()
{
    return m_pimpl->stateVariableHandler;
}

void RDE::UkfState::propagate(const Eigen::Ref<const Eigen::MatrixXd>& cur_states, Eigen::Ref<Eigen::MatrixXd> prop_states)
{
    constexpr auto logPrefix = "[UkfState::propagate]";

    // Check that everything is initialized and set
    if (!m_pimpl->isFinalized)
    {
        log()->error("{} The ukf state is not well initialized.", logPrefix);
        throw std::runtime_error("Error");
    }

    if (m_pimpl->ukfInputProvider == nullptr)
    {
        log()->error("{} The ukf input provider is not set.", logPrefix);
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
            log()->error("{} Cannot update the dynamics with name `{}`.", logPrefix, name);
            throw std::runtime_error("Error");
        }

        m_pimpl->nextState.segment(m_pimpl->stateVariableHandler.getVariable(name).offset,
                                   m_pimpl->stateVariableHandler.getVariable(name).size) = dynamics->getUpdatedVariable();
    }

    prop_states = m_pimpl->nextState;
}

bfl::VectorDescription RDE::UkfState::getStateDescription()
{
    return bfl::VectorDescription(m_pimpl->stateSize);
}

std::size_t RDE::UkfState::getStateSize()
{
    return m_pimpl->stateSize;
}
