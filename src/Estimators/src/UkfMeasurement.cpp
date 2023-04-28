/**
 * @file UkfMeasurement.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <map>

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Math/Wrench.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelKinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/UkfMeasurement.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/ZeroVelocityDynamics.h>

namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion;

using namespace std::chrono;

struct RDE::UkfMeasurement::Impl
{
    bool isInitialized{false};
    bool isFinalized{false};

    bfl::VectorDescription measurementDescription;
    bfl::VectorDescription inputDescription;

    Eigen::Vector3d gravity{0, 0, -Math::StandardAccelerationOfGravitation}; /**< Gravity vector. */

    Eigen::MatrixXd covarianceR; /**< Covariance matrix. */
    int measurementSize{0}; /**< Length of the measurement vector. */
    double dT; /**< Sampling time */

    std::vector<std::pair<std::string, std::shared_ptr<Dynamics>>> dynamicsList;
//    std::map<std::string, std::shared_ptr<Dynamics>> dynamicsList; /**< List of the dynamics composing the process model. */

    System::VariablesHandler measurementVariableHandler; /**< Variable handler describing the measurement vector. */
    System::VariablesHandler stateVariableHandler; /**< Variable handler describing the state vector. */

    std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel; /**< KinDynComputation object for the full model. */
    std::vector<SubModel> subModelList; /**< List of SubModel object describing the sub-models composing the full model. */
    std::vector<std::shared_ptr<SubModelKinDynWrapper>> kinDynWrapperList; /**< List of SubModelKinDynWrapper objects containing kinematic and dynamic information specific of each sub-model. */

    std::shared_ptr<const UkfInputProvider> ukfInputProvider; /**< Provider containing the updated robot state. */
    UKFInput ukfInput; /**< Struct containing the inputs for the ukf populated by the ukfInputProvider. */

    Eigen::VectorXd jointVelocityState; /**< Joint velocity computed by the ukf. */
    Eigen::VectorXd jointAccelerationState; /**< Joint acceleration computed from forward dynamics which depends on the current ukf state. */

    Eigen::VectorXd currentState; /**< State estimated in the previous step. */
    Eigen::VectorXd tempPredictedMeas;
    Eigen::MatrixXd predictedMeasurement; /**< Vector containing the updated measurement. */

    std::vector<Eigen::VectorXd> subModelJointVel; /**< List of sub-model joint velocities. */
    std::vector<Eigen::VectorXd> subModelJointAcc; /**< List of sub-model joint accelerations. */
    std::vector<Eigen::VectorXd> subModelJointMotorTorque; /**< List of sub-model joint velocities. */
    std::vector<Eigen::VectorXd> subModelFrictionTorque; /**< List of sub-model joint velocities. */
    std::map<std::string, Math::Wrenchd> FTMap; /**< The map contains names of the ft sensors and values of the wrench */
    std::map<std::string, Math::Wrenchd> extContactMap; /**< The map contains names of the ft sensors and values of the wrench */
    manif::SE3d::Tangent tempSubModelBaseAcc; /**< Acceleration of the base of the sub-model. */

    // Support variables
    std::vector<Eigen::VectorXd> totalTorqueFromContacts; /**< Joint torques due to known and unknown contacts on the sub-model. */
    std::vector<Eigen::VectorXd> torqueFromContact; /**< Joint torques due to a specific contact. */
    Math::Wrenchd wrench; /**< Joint torques due to a specific contact. */


    Eigen::VectorXd measurement; /**< Measurements coming from the sensors. */

    std::map<std::string, Eigen::VectorXd> measurementMap; /**< Measurement map <measurement name, measurement value>. */

    int offsetMeasurement; /**< Offset used to fill the measurement vector. */

    bool updateRobotDynamicsOnly{false};

    void unpackState()
    {
        jointVelocityState = currentState.segment(stateVariableHandler.getVariable("ds").offset,
                                                  stateVariableHandler.getVariable("ds").size);

        for (int subModelIdx = 0; subModelIdx < subModelList.size(); subModelIdx++)
        {
            // Take sub-model joint velocities, motor torques, friction torques, ft wrenches, ext contact wrenches
            for (int jointIdx = 0; jointIdx < subModelList[subModelIdx].getModel().getNrOfDOFs(); jointIdx++)
            {
                subModelJointVel[subModelIdx](jointIdx) =
                        jointVelocityState(subModelList[subModelIdx].getJointMapping()[jointIdx]);

                subModelJointMotorTorque[subModelIdx](jointIdx) =
                        currentState[stateVariableHandler.getVariable("tau_m").offset +
                        subModelList[subModelIdx].getJointMapping()[jointIdx]];

                subModelFrictionTorque[subModelIdx](jointIdx) =
                        currentState[stateVariableHandler.getVariable("tau_F").offset +
                        subModelList[subModelIdx].getJointMapping()[jointIdx]];
            }

            for (int idx = 0; idx < subModelList[subModelIdx].getNrOfFTSensor(); idx++)
            {
                FTMap[subModelList[subModelIdx].getFTSensor(idx).name] =
                        currentState.segment(stateVariableHandler.getVariable(subModelList[subModelIdx].getFTSensor(idx).name).offset,
                                             stateVariableHandler.getVariable(subModelList[subModelIdx].getFTSensor(idx).name).size);
            }

            for (int idx = 0; idx < subModelList[subModelIdx].getNrOfExternalContact(); idx++)
            {
                extContactMap[subModelList[subModelIdx].getExternalContact(idx)] =
                        currentState.segment(stateVariableHandler.getVariable(subModelList[subModelIdx].getExternalContact(idx)).offset,
                                             stateVariableHandler.getVariable(subModelList[subModelIdx].getExternalContact(idx)).size);
            }
        }
    }

    bool updateState()
    {
        // Update kindyn full model
        kinDynFullModel->setRobotState(ukfInput.robotBasePose.transform(),
                                       ukfInput.robotJointPositions,
                                       iDynTree::make_span(ukfInput.robotBaseVelocity.data(), manif::SE3d::Tangent::DoF),
                                       jointVelocityState,
                                       gravity);

        // compute joint acceleration per each sub-model containing the accelerometer
        for (int subModelIdx = 0; subModelIdx < subModelList.size(); subModelIdx++)
        {
            // Update the kindyn wrapper object of the submodel
            kinDynWrapperList[subModelIdx]->updateState(ukfInput.robotBaseAcceleration,
                                                        jointAccelerationState,
                                                        updateRobotDynamicsOnly);

            if (subModelList[subModelIdx].getModel().getNrOfDOFs() > 0)
            {
                totalTorqueFromContacts[subModelIdx].setZero();

                // Contribution of FT measurements
                for (int idx = 0; idx < subModelList[subModelIdx].getNrOfFTSensor(); idx++)
                {
                    wrench = (int)subModelList[subModelIdx].getFTSensor(idx).forceDirection *
                            FTMap[subModelList[subModelIdx].getFTSensor(idx).name].array();

                    torqueFromContact[subModelIdx] = kinDynWrapperList[subModelIdx]->
                            getFTJacobian(subModelList[subModelIdx].getFTSensor(idx).name).
                            block(0, 6, 6, subModelList[subModelIdx].getModel().getNrOfDOFs()).transpose() * wrench;

                    totalTorqueFromContacts[subModelIdx] = totalTorqueFromContacts[subModelIdx].array() + torqueFromContact[subModelIdx].array();
                }

                // Contribution of unknown external contacts
                for (int idx = 0; idx < subModelList[subModelIdx].getNrOfExternalContact(); idx++)
                {
                    torqueFromContact[subModelIdx] = kinDynWrapperList[subModelIdx]->getExtContactJacobian(subModelList[subModelIdx].getExternalContact(idx)).block(0, 6, 6, subModelList[subModelIdx].getModel().getNrOfDOFs()).transpose() * extContactMap[subModelList[subModelIdx].getExternalContact(idx)];

                    totalTorqueFromContacts[subModelIdx] = totalTorqueFromContacts[subModelIdx].array() + torqueFromContact[subModelIdx].array();
                }

                tempSubModelBaseAcc = kinDynWrapperList[subModelIdx]->getBaseAcceleration();

                if (!kinDynWrapperList[subModelIdx]->forwardDynamics(subModelJointMotorTorque[subModelIdx],
                                                                     subModelFrictionTorque[subModelIdx],
                                                                     totalTorqueFromContacts[subModelIdx],
                                                                     tempSubModelBaseAcc.coeffs(),
                                                                     subModelJointAcc[subModelIdx]))
                {
                    BipedalLocomotion::log()->error("Cannot compute the inverse dynamics.");
                    return false;
                }

                // Assign joint acceleration using the correct indeces
                for (int jointIdx = 0; jointIdx < subModelList[subModelIdx].getJointMapping().size(); jointIdx++)
                {
                    jointAccelerationState[subModelList[subModelIdx].getJointMapping()[jointIdx]] = subModelJointAcc[subModelIdx][jointIdx];
                }
            }
        }

        return true;
    }
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
    for (int indexDyn1 = 0; indexDyn1 <  m_pimpl->dynamicsList.size(); indexDyn1++)
    {
        if(!m_pimpl->dynamicsList[indexDyn1].second->finalize(handler))
        {
            BipedalLocomotion::log()->error("{} Error while finalizing the dynamics named {}", logPrefix, m_pimpl->dynamicsList[indexDyn1].first);
            return false;
        }

        m_pimpl->measurementSize += m_pimpl->dynamicsList[indexDyn1].second->size();
    }

    // Set value of measurement covariance matrix
    m_pimpl->covarianceR.resize(m_pimpl->measurementSize, m_pimpl->measurementSize);
    m_pimpl->covarianceR.setZero();

    for (int indexDyn2 = 0; indexDyn2 < m_pimpl->dynamicsList.size(); indexDyn2++)
    {
        m_pimpl->measurementVariableHandler.addVariable(m_pimpl->dynamicsList[indexDyn2].first, m_pimpl->dynamicsList[indexDyn2].second->getCovariance().size());

        m_pimpl->covarianceR.block(m_pimpl->measurementVariableHandler.getVariable(m_pimpl->dynamicsList[indexDyn2].first).offset, m_pimpl->measurementVariableHandler.getVariable(m_pimpl->dynamicsList[indexDyn2].first).offset,
                                   m_pimpl->measurementVariableHandler.getVariable(m_pimpl->dynamicsList[indexDyn2].first).size, m_pimpl->measurementVariableHandler.getVariable(m_pimpl->dynamicsList[indexDyn2].first).size) = m_pimpl->dynamicsList[indexDyn2].second->getCovariance().asDiagonal();
    }

    m_pimpl->jointVelocityState.resize(m_pimpl->kinDynFullModel->model().getNrOfDOFs());
    m_pimpl->jointAccelerationState.resize(m_pimpl->kinDynFullModel->model().getNrOfDOFs());

    for (int idx = 0; idx < m_pimpl->subModelList.size(); idx++)
    {
        m_pimpl->subModelJointVel.emplace_back(Eigen::VectorXd(m_pimpl->subModelList[idx].getModel().getNrOfDOFs()));
        m_pimpl->subModelJointAcc.emplace_back(Eigen::VectorXd(m_pimpl->subModelList[idx].getModel().getNrOfDOFs()));
        m_pimpl->subModelJointMotorTorque.emplace_back(Eigen::VectorXd(m_pimpl->subModelList[idx].getModel().getNrOfDOFs()));
        m_pimpl->subModelFrictionTorque.emplace_back(Eigen::VectorXd(m_pimpl->subModelList[idx].getModel().getNrOfDOFs()));
        m_pimpl->totalTorqueFromContacts.emplace_back(Eigen::VectorXd(m_pimpl->subModelList[idx].getModel().getNrOfDOFs()));
        m_pimpl->torqueFromContact.emplace_back(Eigen::VectorXd(m_pimpl->subModelList[idx].getModel().getNrOfDOFs()));
    }

    m_pimpl->currentState.resize(m_pimpl->measurementSize);
    m_pimpl->currentState.setZero();

    m_pimpl->measurement.resize(m_pimpl->measurementSize);
    m_pimpl->measurement.setZero();

    m_pimpl->tempPredictedMeas.resize(m_pimpl->measurementSize);

    m_pimpl->measurementDescription = bfl::VectorDescription(m_pimpl->measurementSize);

    m_pimpl->predictedMeasurement.resize(m_pimpl->measurementSize, 2*m_pimpl->stateVariableHandler.getNumberOfVariables()+1);

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
//        measurement->m_pimpl->dynamicsList.insert({dynamicsName, dynamicsInstance});
        measurement->m_pimpl->dynamicsList.emplace_back(dynamicsName, dynamicsInstance);
    }

    measurement->m_pimpl->inputDescription = bfl::VectorDescription(stateVariableHandler.getNumberOfVariables());

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
    return m_pimpl->measurementDescription;
}

bfl::VectorDescription RDE::UkfMeasurement::getInputDescription() const
{
    return m_pimpl->inputDescription;
}

// TODO
// Here the cur_state has size state_size x n_sigma_points
// this means that the computation can be parallelized
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

    for (int sample = 0; sample < cur_states.cols(); sample++)
    {
        m_pimpl->currentState = cur_states.block(0, sample, cur_states.rows(), 1);;

        m_pimpl->unpackState();

        // Update kindyn full model
        m_pimpl->kinDynFullModel->setRobotState(m_pimpl->ukfInput.robotBasePose.transform(),
                                                m_pimpl->ukfInput.robotJointPositions,
                                                iDynTree::make_span(m_pimpl->ukfInput.robotBaseVelocity.data(), manif::SE3d::Tangent::DoF),
                                                m_pimpl->jointVelocityState,
                                                m_pimpl->gravity);

        if (!m_pimpl->updateState())
        {
            BipedalLocomotion::log()->error("{} The joint accelerations are not updated.", logPrefix);
            throw std::runtime_error("Error");
        }

        m_pimpl->ukfInput.robotJointAccelerations = m_pimpl->jointAccelerationState;

        // TODO
        // This could be parallelized

        // Update all the dynamics
//        for (auto& [name, dynamics] : m_pimpl->dynamicsList)
//        {
//            std::cout << "variable name = " << name << " , variable offset = " << m_pimpl->measurementVariableHandler.getVariable(name).offset << std::endl;

//            dynamics->setState(m_pimpl->currentState);

//            dynamics->setInput(m_pimpl->ukfInput);

//            if (!dynamics->update())
//            {
//                BipedalLocomotion::log()->error("{} Cannot update the dynamics with name `{}`.", logPrefix, name);
//                throw std::runtime_error("Error");
//            }

//            m_pimpl->tempPredictedMeas.segment(m_pimpl->measurementVariableHandler.getVariable(name).offset,
//                                                  m_pimpl->measurementVariableHandler.getVariable(name).size) = dynamics->getUpdatedVariable();
//        }
        for (int indexDyn = 0; indexDyn < m_pimpl->dynamicsList.size(); indexDyn++)
        {
            m_pimpl->dynamicsList[indexDyn].second->setState(m_pimpl->currentState);

            m_pimpl->dynamicsList[indexDyn].second->setInput(m_pimpl->ukfInput);

            if (!m_pimpl->dynamicsList[indexDyn].second->update())
            {
                BipedalLocomotion::log()->error("{} Cannot update the dynamics with name `{}`.", logPrefix, m_pimpl->dynamicsList[indexDyn].first);
                throw std::runtime_error("Error");
            }

            m_pimpl->tempPredictedMeas.segment(m_pimpl->measurementVariableHandler.getVariable(m_pimpl->dynamicsList[indexDyn].first).offset,
                                                  m_pimpl->measurementVariableHandler.getVariable(m_pimpl->dynamicsList[indexDyn].first).size) = m_pimpl->dynamicsList[indexDyn].second->getUpdatedVariable();
        }
        m_pimpl->predictedMeasurement.block(0, sample, m_pimpl->measurementSize, 1) = m_pimpl->tempPredictedMeas;
    }

    return std::make_pair(true, m_pimpl->predictedMeasurement);
}

std::pair<bool, bfl::Data> RDE::UkfMeasurement::innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const
{
    Eigen::MatrixXd innovation = -(bfl::any::any_cast<Eigen::MatrixXd>(predicted_measurements).colwise() - bfl::any::any_cast<Eigen::VectorXd>(measurements));

    return std::make_pair(true, std::move(innovation));
}

std::size_t RDE::UkfMeasurement::getMeasurementSize()
{
    return m_pimpl->measurementSize;
}

BipedalLocomotion::System::VariablesHandler& RDE::UkfMeasurement::getMeasurementVariableHandler()
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
    constexpr auto logPrefix = "[UkfMeasurement::freeze]";

    m_pimpl->measurementMap = bfl::any::any_cast<std::map<std::string, Eigen::VectorXd>>(data);

//    for (auto& [name, dynamics] : m_pimpl->dynamicsList)
//    {
//        m_pimpl->offsetMeasurement = m_pimpl->measurementVariableHandler.getVariable(name).offset;

//        if(m_pimpl->measurementMap.count(name) == 0)
//        {
//            BipedalLocomotion::log()->error("{} Measurement with name `{}` not found.", logPrefix, name);
//            return false;
//        }

//        // If more sub-models share the same accelerometer or gyroscope sensor, the measurement vector is concatenated
//        // a number of times equal to the number of sub-models using the sensor.
//        while(m_pimpl->offsetMeasurement <
//              (m_pimpl->measurementVariableHandler.getVariable(name).offset + m_pimpl->measurementVariableHandler.getVariable(name).size))
//        {
//            m_pimpl->measurement.segment(m_pimpl->offsetMeasurement, m_pimpl->measurementMap[name].size())
//                    = m_pimpl->measurementMap[name];

//            m_pimpl->offsetMeasurement += m_pimpl->measurementMap[name].size();
//        }
//    }
    for (int index = 0; index < m_pimpl->dynamicsList.size(); index++)
    {
        m_pimpl->offsetMeasurement = m_pimpl->measurementVariableHandler.getVariable(m_pimpl->dynamicsList[index].first).offset;

        if(m_pimpl->measurementMap.count(m_pimpl->dynamicsList[index].first) == 0)
        {
            BipedalLocomotion::log()->error("{} Measurement with name `{}` not found.", logPrefix, m_pimpl->dynamicsList[index].first);
            return false;
        }

        // If more sub-models share the same accelerometer or gyroscope sensor, the measurement vector is concatenated
        // a number of times equal to the number of sub-models using the sensor.
        while(m_pimpl->offsetMeasurement <
              (m_pimpl->measurementVariableHandler.getVariable(m_pimpl->dynamicsList[index].first).offset + m_pimpl->measurementVariableHandler.getVariable(m_pimpl->dynamicsList[index].first).size))
        {
            m_pimpl->measurement.segment(m_pimpl->offsetMeasurement, m_pimpl->measurementMap[m_pimpl->dynamicsList[index].first].size())
                    = m_pimpl->measurementMap[m_pimpl->dynamicsList[index].first];

            m_pimpl->offsetMeasurement += m_pimpl->measurementMap[m_pimpl->dynamicsList[index].first].size();
        }
    }

    return true;
}
