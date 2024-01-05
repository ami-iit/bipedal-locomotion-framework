/**
 * @file UkfState.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <map>

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/FrictionTorqueStateDynamics.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/JointVelocityStateDynamics.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/KinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/UkfState.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/ZeroVelocityStateDynamics.h>

using namespace BipedalLocomotion;
namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

using namespace std::chrono;

bool RDE::UkfState::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[UkfState::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("sampling_time", m_dT))
    {
        log()->error("{} Unable to find the `sampling_time` variable", logPrefix);
        return false;
    }

    m_isInitialized = true;

    return true;
}

bool RDE::UkfState::finalize(const System::VariablesHandler& handler)
{
    constexpr auto logPrefix = "[UkfState::finalize]";

    if (!m_isInitialized)
    {
        log()->error("{} Please call initialize() before finalize().", logPrefix);
        return false;
    }

    m_isFinalized = false;

    m_stateSize = 0;

    for (int indexDyn1 = 0; indexDyn1 < m_dynamicsList.size(); indexDyn1++)
    {
        if (!m_dynamicsList[indexDyn1].second->finalize(handler))
        {
            log()->error("{} Error while finalizing the dynamics named {}",
                         logPrefix,
                         m_dynamicsList[indexDyn1].first);
            return false;
        }

        m_stateSize += m_dynamicsList[indexDyn1].second->size();
    }

    // Set value of process covariance matrix
    m_covarianceQ.resize(m_stateSize, m_stateSize);
    m_covarianceQ.setZero();

    m_initialCovariance.resize(m_stateSize, m_stateSize);
    m_initialCovariance.setZero();

    for (int indexDyn2 = 0; indexDyn2 < m_dynamicsList.size(); indexDyn2++)
    {
        m_covarianceQ.block(handler.getVariable(m_dynamicsList[indexDyn2].first).offset,
                            handler.getVariable(m_dynamicsList[indexDyn2].first).offset,
                            handler.getVariable(m_dynamicsList[indexDyn2].first).size,
                            handler.getVariable(m_dynamicsList[indexDyn2].first).size)
            = m_dynamicsList[indexDyn2].second->getCovariance().asDiagonal();

        m_initialCovariance.block(handler.getVariable(m_dynamicsList[indexDyn2].first).offset,
                                  handler.getVariable(m_dynamicsList[indexDyn2].first).offset,
                                  handler.getVariable(m_dynamicsList[indexDyn2].first).size,
                                  handler.getVariable(m_dynamicsList[indexDyn2].first).size)
            = m_dynamicsList[indexDyn2].second->getInitialStateCovariance().asDiagonal();
    }

    m_jointVelocityState.resize(m_kinDynFullModel->model().getNrOfDOFs());
    m_jointAccelerationState.resize(m_kinDynFullModel->model().getNrOfDOFs());

    for (int idx = 0; idx < m_subModelList.size(); idx++)
    {
        m_subModelJointVel.emplace_back(
            Eigen::VectorXd(m_subModelList[idx].getModel().getNrOfDOFs()));
        m_subModelJointAcc.emplace_back(
            Eigen::VectorXd(m_subModelList[idx].getModel().getNrOfDOFs()));
        m_subModelJointPos.emplace_back(
            Eigen::VectorXd(m_subModelList[idx].getModel().getNrOfDOFs()));
        m_subModelJointMotorTorque.emplace_back(
            Eigen::VectorXd(m_subModelList[idx].getModel().getNrOfDOFs()));
        m_subModelFrictionTorque.emplace_back(
            Eigen::VectorXd(m_subModelList[idx].getModel().getNrOfDOFs()));
        m_totalTorqueFromContacts.emplace_back(
            Eigen::VectorXd(6 + m_subModelList[idx].getModel().getNrOfDOFs()));
        m_tempJacobianList.emplace_back(
            Eigen::MatrixXd(6, 6 + m_subModelList[idx].getModel().getNrOfDOFs()));
        m_subModelNuDot.emplace_back(
            Eigen::VectorXd(6 + m_subModelList[idx].getModel().getNrOfDOFs()));
    }

    m_currentState.resize(m_stateSize);
    m_currentState.setZero();

    m_nextState.resize(m_stateSize);
    m_nextState.setZero();

    m_isFinalized = true;

    return true;
}

std::unique_ptr<RDE::UkfState>
RDE::UkfState::build(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                     std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel,
                     const std::vector<SubModel>& subModelList,
                     const std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList)
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
        log()->error("{} Unable to initialize the state object of the ukf.", logPrefix);
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
    state->m_kinDynFullModel = kinDynFullModel;

    // Set the list of SubModel
    state->m_subModelList.reserve(subModelList.size());
    state->m_subModelList = subModelList;

    // set the list of KinDynWrapper
    state->m_kinDynWrapperList.reserve(kinDynWrapperList.size());
    state->m_kinDynWrapperList = kinDynWrapperList;

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
        dynamicsGroup->setParameter("sampling_time", state->m_dT);

        // create variable handler
        std::string inputName;
        if (!dynamicsGroup->getParameter("input_name", inputName))
        {
            log()->error("{} Unable to find the parameter 'input_name'.", logPrefix);
            return nullptr;
        }
        std::vector<double> covariances;
        if (!dynamicsGroup->getParameter("covariance", covariances))
        {
            log()->error("{} Unable to find the parameter 'covariance'.", logPrefix);
            return nullptr;
        }
        if (!dynamicsGroup->getParameter("initial_covariance", covariances))
        {
            log()->error("{} Unable to find the parameter 'initial_covariance'.", logPrefix);
            return nullptr;
        }
        state->m_stateVariableHandler.addVariable(dynamicsGroupName, covariances.size());

        std::string dynamicModel;
        if (!dynamicsGroup->getParameter("dynamic_model", dynamicModel))
        {
            log()->error("{} Unable to find the parameter 'dynamic_model'.", logPrefix);
            return nullptr;
        }

        std::shared_ptr<Dynamics> dynamicsInstance
            = RDE::DynamicsFactory::createInstance(dynamicModel);
        if (dynamicsInstance == nullptr)
        {
            log()->error("{} The dynamic model '{}' has not been registered for the state variable "
                         "`{}`.",
                         logPrefix,
                         dynamicModel,
                         dynamicsGroupName);
            return nullptr;
        }

        dynamicsInstance->setSubModels(subModelList, kinDynWrapperList);

        dynamicsInstance->initialize(dynamicsGroup, dynamicsGroupName);

        // add dynamics to the list
        state->m_dynamicsList.emplace_back(dynamicsGroupName, dynamicsInstance);

        state->m_stateToUkfNames[inputName] = dynamicsGroupName;
    }

    // finalize estimator
    if (!state->finalize(state->m_stateVariableHandler))
    {
        log()->error("{} Unable to finalize the RobotDynamicsEstimator.", logPrefix);
        return nullptr;
    }

    return state;
}

void RDE::UkfState::setUkfInputProvider(std::shared_ptr<const UkfInputProvider> ukfInputProvider)
{
    m_ukfInputProvider = ukfInputProvider;
}

Eigen::MatrixXd RDE::UkfState::getNoiseCovarianceMatrix()
{
    return m_covarianceQ;
}

const System::VariablesHandler& RDE::UkfState::getStateVariableHandler() const
{
    return m_stateVariableHandler;
}

void RDE::UkfState::propagate(const Eigen::Ref<const Eigen::MatrixXd>& currentStates,
                              Eigen::Ref<Eigen::MatrixXd> propagatedStates)
{
    constexpr auto logPrefix = "[UkfState::propagate]";

    // Check that everything is initialized and set
    if (!m_isFinalized)
    {
        log()->error("{} The ukf state is not well initialized.", logPrefix);
        throw std::runtime_error("Error");
    }

    if (m_ukfInputProvider == nullptr)
    {
        log()->error("{} The ukf input provider is not set.", logPrefix);
        throw std::runtime_error("Error");
    }

    // Get input of ukf from provider
    m_ukfInput = m_ukfInputProvider->getOutput();

    propagatedStates.resize(currentStates.rows(), currentStates.cols());

    for (int sample = 0; sample < currentStates.cols(); sample++)
    {
        m_currentState = currentStates.col(sample);

        unpackState();

        if (!updateState())
        {
            log()->error("{} The joint accelerations are not updated.", logPrefix);
            throw std::runtime_error("Error");
        }

        m_ukfInput.robotJointAccelerations = m_jointAccelerationState;

        for (int indexDyn = 0; indexDyn < m_dynamicsList.size(); indexDyn++)
        {
            m_dynamicsList[indexDyn].second->setState(m_currentState);

            m_dynamicsList[indexDyn].second->setInput(m_ukfInput);

            if (!m_dynamicsList[indexDyn].second->update())
            {
                log()->error("{} Cannot update the dynamics with name `{}`.",
                             logPrefix,
                             m_dynamicsList[indexDyn].first);
                throw std::runtime_error("Error");
            }

            m_nextState
                .segment(m_stateVariableHandler.getVariable(m_dynamicsList[indexDyn].first).offset,
                         m_stateVariableHandler.getVariable(m_dynamicsList[indexDyn].first).size)
                = m_dynamicsList[indexDyn].second->getUpdatedVariable();
        }

        propagatedStates.col(sample) = m_nextState;
    }
}

bfl::VectorDescription RDE::UkfState::getStateDescription()
{
    return bfl::VectorDescription(m_stateSize);
}

std::size_t RDE::UkfState::getStateSize()
{
    return m_stateSize;
}

Eigen::Ref<const Eigen::MatrixXd> RDE::UkfState::getInitialStateCovarianceMatrix() const
{
    return m_initialCovariance;
}

bool RDE::UkfState::setProperty(const std::string& property)
{
    return false;
}
