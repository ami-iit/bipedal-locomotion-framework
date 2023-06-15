/**
 * @file UkfState.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <map>

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/FrictionTorqueStateDynamics.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/JointVelocityStateDynamics.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelKinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/UkfState.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/ZeroVelocityStateDynamics.h>

using namespace BipedalLocomotion;
namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

using namespace std::chrono;

struct RDE::UkfState::Impl
{
    bool isInitialized{false};
    bool isFinalized{false};

    Eigen::Vector3d gravity{0, 0, -Math::StandardAccelerationOfGravitation}; /**< Gravity vector. */

    Eigen::MatrixXd covarianceQ; /**< Covariance matrix. */
    Eigen::MatrixXd initialCovariance; /**< Initial covariance matrix. */
    std::size_t stateSize; /**< Length of the state vector. */
    std::chrono::nanoseconds dT; /**< Sampling time */

    std::vector<std::pair<std::string, std::shared_ptr<Dynamics>>> dynamicsList; /**< List of the
                                                                                    dynamics
                                                                                    composing the
                                                                                    process model.
                                                                                  */

    System::VariablesHandler stateVariableHandler; /**< Variable handler describing the state
                                                      vector. */

    std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel; /**< KinDynComputation object for
                                                                      the full model. */
    std::vector<SubModel> subModelList; /**< List of SubModel object describing the sub-models
                                           composing the full model. */
    std::vector<std::shared_ptr<SubModelKinDynWrapper>> kinDynWrapperList; /**< List of
                                                                              SubModelKinDynWrapper
                                                                              objects containing
                                                                              kinematic and dynamic
                                                                              information specific
                                                                              of each sub-model. */

    std::shared_ptr<const UkfInputProvider> ukfInputProvider; /**< Provider containing the updated
                                                                 robot state. */
    UKFInput ukfInput; /**< Struct containing the inputs for the ukf populated by the
                          ukfInputProvider. */

    Eigen::VectorXd jointVelocityState; /**< Joint velocity computed by the ukf. */
    Eigen::VectorXd jointAccelerationState; /**< Joint acceleration computed from forward dynamics
                                               which depends on the current ukf state. */
    Eigen::VectorXd currentState; /**< State estimated in the previous step. */
    Eigen::VectorXd nextState; /**< Vector containing all the updated states. */

    std::vector<Eigen::VectorXd> subModelJointVel; /**< List of sub-model joint velocities. */
    std::vector<Eigen::VectorXd> subModelJointAcc; /**< List of sub-model joint accelerations. */
    std::vector<Eigen::VectorXd> subModelJointMotorTorque; /**< List of sub-model joint motor
                                                              torques. */
    std::vector<Eigen::VectorXd> subModelFrictionTorque; /**< List of sub-model friction torques. */
    std::map<std::string, Math::Wrenchd> FTMap; /**< The map contains names of the ft sensors and
                                                   values of the wrench */
    std::map<std::string, Math::Wrenchd> extContactMap; /**< The map contains names of the ft
                                                           sensors and values of the wrench */
    manif::SE3d::Tangent tempSubModelBaseAcc; /**< Acceleration of the base of the sub-model. */

    // Support variables
    std::vector<Eigen::VectorXd> totalTorqueFromContacts; /**< Joint torques due to known and
                                                             unknown contacts on the sub-model. */
    std::vector<Eigen::VectorXd> torqueFromContact; /**< Joint torques due to a specific contact. */
    Math::Wrenchd wrench; /**< Joint torques due to a specific contact. */

    bool updateRobotDynamicsOnly{true};

    void unpackState()
    {
        jointVelocityState = currentState.segment(stateVariableHandler.getVariable("ds").offset,
                                                  stateVariableHandler.getVariable("ds").size);

        for (int subModelIdx = 0; subModelIdx < subModelList.size(); subModelIdx++)
        {
            // Take sub-model joint velocities, motor torques, friction torques, ft wrenches, ext
            // contact wrenches
            for (int jointIdx = 0; jointIdx < subModelList[subModelIdx].getModel().getNrOfDOFs();
                 jointIdx++)
            {
                subModelJointVel[subModelIdx](jointIdx)
                    = jointVelocityState(subModelList[subModelIdx].getJointMapping()[jointIdx]);

                subModelJointMotorTorque[subModelIdx](jointIdx)
                    = currentState[stateVariableHandler.getVariable("tau_m").offset
                                   + subModelList[subModelIdx].getJointMapping()[jointIdx]];

                subModelFrictionTorque[subModelIdx](jointIdx)
                    = currentState[stateVariableHandler.getVariable("tau_F").offset
                                   + subModelList[subModelIdx].getJointMapping()[jointIdx]];
            }

            for (const auto& [key, value] : subModelList[subModelIdx].getFTList())
            {
                FTMap[key] = currentState.segment(stateVariableHandler.getVariable(key).offset,
                                                  stateVariableHandler.getVariable(key).size);
            }

            for (int idx = 0; idx < subModelList[subModelIdx].getNrOfExternalContact(); idx++)
            {
                extContactMap[subModelList[subModelIdx].getExternalContact(idx)]
                    = currentState
                          .segment(stateVariableHandler
                                       .getVariable(
                                           subModelList[subModelIdx].getExternalContact(idx))
                                       .offset,
                                   stateVariableHandler
                                       .getVariable(
                                           subModelList[subModelIdx].getExternalContact(idx))
                                       .size);
            }
        }
    }

    bool updateState()
    {
        // Update kindyn full model
        kinDynFullModel->setRobotState(ukfInput.robotBasePose.transform(),
                                       ukfInput.robotJointPositions,
                                       iDynTree::make_span(ukfInput.robotBaseVelocity.data(),
                                                           manif::SE3d::Tangent::DoF),
                                       jointVelocityState,
                                       gravity);

        // compute joint acceleration per each sub-model
        for (int subModelIdx = 0; subModelIdx < subModelList.size(); subModelIdx++)
        {
            // Update the kindyn wrapper object of the submodel
            kinDynWrapperList[subModelIdx]->updateState(ukfInput.robotBaseAcceleration,
                                                        jointAccelerationState,
                                                        UpdateMode::RobotDynamicsOnly);

            if (subModelList[subModelIdx].getModel().getNrOfDOFs() > 0)
            {
                totalTorqueFromContacts[subModelIdx].setZero();

                // Contribution of FT measurements
                for (const auto& [key, value] : subModelList[subModelIdx].getFTList())
                {
                    wrench.noalias() = static_cast<int>(value.forceDirection) * FTMap[key];

                    torqueFromContact[subModelIdx].noalias()
                        = kinDynWrapperList[subModelIdx]
                              ->getFTJacobian(key)
                              .block(0, 6, 6, subModelList[subModelIdx].getModel().getNrOfDOFs())
                              .transpose()
                          * wrench;

                    totalTorqueFromContacts[subModelIdx]
                        = totalTorqueFromContacts[subModelIdx] + torqueFromContact[subModelIdx];
                }

                // Contribution of unknown external contacts
                for (int idx = 0; idx < subModelList[subModelIdx].getNrOfExternalContact(); idx++)
                {
                    torqueFromContact[subModelIdx].noalias()
                        = kinDynWrapperList[subModelIdx]
                              ->getExtContactJacobian(
                                  subModelList[subModelIdx].getExternalContact(idx))
                              .block(0, 6, 6, subModelList[subModelIdx].getModel().getNrOfDOFs())
                              .transpose()
                          * extContactMap[subModelList[subModelIdx].getExternalContact(idx)];

                    totalTorqueFromContacts[subModelIdx]
                        = totalTorqueFromContacts[subModelIdx] + torqueFromContact[subModelIdx];
                }

                tempSubModelBaseAcc = kinDynWrapperList[subModelIdx]->getBaseAcceleration();

                if (!kinDynWrapperList[subModelIdx]
                         ->forwardDynamics(subModelJointMotorTorque[subModelIdx],
                                           subModelFrictionTorque[subModelIdx],
                                           totalTorqueFromContacts[subModelIdx],
                                           tempSubModelBaseAcc.coeffs(),
                                           subModelJointAcc[subModelIdx]))
                {
                    log()->error("Cannot compute the inverse dynamics.");
                    return false;
                }

                // Assign joint acceleration using the correct indeces
                for (int jointIdx = 0;
                     jointIdx < subModelList[subModelIdx].getJointMapping().size();
                     jointIdx++)
                {
                    jointAccelerationState[subModelList[subModelIdx].getJointMapping()[jointIdx]]
                        = subModelJointAcc[subModelIdx][jointIdx];
                }
            }
        }

        return true;
    }
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
    for (int indexDyn1 = 0; indexDyn1 < m_pimpl->dynamicsList.size(); indexDyn1++)
    {
        if (!m_pimpl->dynamicsList[indexDyn1].second->finalize(handler))
        {
            log()->error("{} Error while finalizing the dynamics named {}",
                         logPrefix,
                         m_pimpl->dynamicsList[indexDyn1].first);
            return false;
        }

        m_pimpl->stateSize += m_pimpl->dynamicsList[indexDyn1].second->size();
    }

    // Set value of process covariance matrix
    m_pimpl->covarianceQ.resize(m_pimpl->stateSize, m_pimpl->stateSize);
    m_pimpl->covarianceQ.setZero();

    m_pimpl->initialCovariance.resize(m_pimpl->stateSize, m_pimpl->stateSize);
    m_pimpl->initialCovariance.setZero();

    for (int indexDyn2 = 0; indexDyn2 < m_pimpl->dynamicsList.size(); indexDyn2++)
    {
        m_pimpl->covarianceQ
            .block(handler.getVariable(m_pimpl->dynamicsList[indexDyn2].first).offset,
                   handler.getVariable(m_pimpl->dynamicsList[indexDyn2].first).offset,
                   handler.getVariable(m_pimpl->dynamicsList[indexDyn2].first).size,
                   handler.getVariable(m_pimpl->dynamicsList[indexDyn2].first).size)
            = m_pimpl->dynamicsList[indexDyn2].second->getCovariance().asDiagonal();

        m_pimpl->initialCovariance
            .block(handler.getVariable(m_pimpl->dynamicsList[indexDyn2].first).offset,
                   handler.getVariable(m_pimpl->dynamicsList[indexDyn2].first).offset,
                   handler.getVariable(m_pimpl->dynamicsList[indexDyn2].first).size,
                   handler.getVariable(m_pimpl->dynamicsList[indexDyn2].first).size)
            = m_pimpl->dynamicsList[indexDyn2].second->getInitialStateCovariance().asDiagonal();
    }

    m_pimpl->jointVelocityState.resize(m_pimpl->kinDynFullModel->model().getNrOfDOFs());
    m_pimpl->jointAccelerationState.resize(m_pimpl->kinDynFullModel->model().getNrOfDOFs());

    for (int idx = 0; idx < m_pimpl->subModelList.size(); idx++)
    {
        m_pimpl->subModelJointVel.emplace_back(m_pimpl->subModelList[idx].getModel().getNrOfDOFs());
        m_pimpl->subModelJointAcc.emplace_back(m_pimpl->subModelList[idx].getModel().getNrOfDOFs());
        m_pimpl->subModelJointMotorTorque.emplace_back(
            m_pimpl->subModelList[idx].getModel().getNrOfDOFs());
        m_pimpl->subModelFrictionTorque.emplace_back(
            m_pimpl->subModelList[idx].getModel().getNrOfDOFs());
        m_pimpl->totalTorqueFromContacts.emplace_back(
            m_pimpl->subModelList[idx].getModel().getNrOfDOFs());
        m_pimpl->torqueFromContact.emplace_back(
            m_pimpl->subModelList[idx].getModel().getNrOfDOFs());
    }

    m_pimpl->currentState.resize(m_pimpl->stateSize);
    m_pimpl->currentState.setZero();

    m_pimpl->nextState.resize(m_pimpl->stateSize);
    m_pimpl->nextState.setZero();

    m_pimpl->isFinalized = true;

    return true;
}

std::unique_ptr<RDE::UkfState>
RDE::UkfState::build(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
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
        if (!dynamicsGroup->getParameter("initial_covariance", covariances))
        {
            log()->error("{} Unable to find the parameter 'initial_covariance'.", logPrefix);
            return nullptr;
        }
        state->m_pimpl->stateVariableHandler.addVariable(dynamicsName, covariances.size());

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
            log()->error("{} The dynamic model '{}' has not been registered.",
                         logPrefix,
                         dynamicModel);
            return nullptr;
        }

        dynamicsInstance->setSubModels(subModelList, kinDynWrapperList);

        dynamicsInstance->initialize(dynamicsGroup);

        // add dynamics to the list
        state->m_pimpl->dynamicsList.emplace_back(dynamicsName, dynamicsInstance);
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

System::VariablesHandler& RDE::UkfState::getStateVariableHandler()
{
    return m_pimpl->stateVariableHandler;
}

void RDE::UkfState::propagate(const Eigen::Ref<const Eigen::MatrixXd>& curStates,
                              Eigen::Ref<Eigen::MatrixXd> propStates)
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

    propStates.resize(curStates.rows(), curStates.cols());

    for (int sample = 0; sample < curStates.cols(); sample++)
    {
        m_pimpl->currentState = curStates.block(0, sample, curStates.rows(), 1);

        m_pimpl->unpackState();

        if (!m_pimpl->updateState())
        {
            log()->error("{} The joint accelerations are not updated.", logPrefix);
            throw std::runtime_error("Error");
        }

        m_pimpl->ukfInput.robotJointAccelerations = m_pimpl->jointAccelerationState;

        // TODO (ines) Investigate if the following code can be parallelized
        for (int indexDyn = 0; indexDyn < m_pimpl->dynamicsList.size(); indexDyn++)
        {
            m_pimpl->dynamicsList[indexDyn].second->setState(m_pimpl->currentState);

            m_pimpl->dynamicsList[indexDyn].second->setInput(m_pimpl->ukfInput);

            if (!m_pimpl->dynamicsList[indexDyn].second->update())
            {
                log()->error("{} Cannot update the dynamics with name `{}`.",
                             logPrefix,
                             m_pimpl->dynamicsList[indexDyn].first);
                throw std::runtime_error("Error");
            }

            m_pimpl->nextState.segment(m_pimpl->stateVariableHandler
                                           .getVariable(m_pimpl->dynamicsList[indexDyn].first)
                                           .offset,
                                       m_pimpl->stateVariableHandler
                                           .getVariable(m_pimpl->dynamicsList[indexDyn].first)
                                           .size)
                = m_pimpl->dynamicsList[indexDyn].second->getUpdatedVariable();
        }

        propStates.block(0, sample, curStates.rows(), 1) = m_pimpl->nextState;
    }
}

bfl::VectorDescription RDE::UkfState::getStateDescription()
{
    return bfl::VectorDescription(m_pimpl->stateSize);
}

std::size_t RDE::UkfState::getStateSize() const
{
    return m_pimpl->stateSize;
}

Eigen::Ref<const Eigen::MatrixXd> RDE::UkfState::getInitialStateCovarianceMatrix()
{
    return m_pimpl->initialCovariance;
}
