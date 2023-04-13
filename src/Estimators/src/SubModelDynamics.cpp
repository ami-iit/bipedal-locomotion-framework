/**
 * @file SubModelDynamics.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <map>
#include <numeric>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelDynamics.h>

namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

RDE::SubModelDynamics::SubModelDynamics() = default;

RDE::SubModelDynamics::~SubModelDynamics() = default;

/**
 * Set the SubModelKinDynWrapper object.
 * Parameter kinDynWrapper pointer to a SubModelKinDynWrapper object.
 * Return true in case of success, false otherwise.
 */
bool RDE::SubModelDynamics::setKinDynWrapper(std::shared_ptr<SubModelKinDynWrapper> kinDynWrapper)
{
    constexpr auto errorPrefix = "[SubModelDynamics::setKinDynWrapper]";

    this->kinDynWrapper = kinDynWrapper;

    if (kinDynWrapper == nullptr)
    {
        log()->error("{} Error while setting the `SubModelKinDynWrapper` object.", errorPrefix);
        return false;
    }

    return true;
}

bool RDE::SubModelDynamics::setSubModel(const RDE::SubModel& subModel)
{
    constexpr auto errorPrefix = "[SubModelDynamics::setSubModel]";

    this->subModel = subModel;

    if (!subModel.isValid())
    {
        log()->error("{} Set a valid `SubModel` object.", errorPrefix);
        return false;
    }

    return true;
}


bool RDE::SubModelDynamics::initialize()
{
    constexpr auto errorPrefix = "[SubModelDynamics::initialize]";

    // Inialize map of the ft sensors
    if (!subModel.isValid())
    {
        log()->error("{} Set a valid `SubModel` object before calling initialize.", errorPrefix);
        return false;
    }

    for (int idx = 0; idx < subModel.getNrOfFTSensor(); idx++)
    {
        FTMap[subModel.getFTSensor(idx).name].setZero();
    }

    // Inialize map of the external contacts
    for (int idx = 0; idx < subModel.getNrOfExternalContact(); idx++)
    {
        extContactMap[subModel.getExternalContact(idx)].setZero();
    }

    // Resize and initialize variables
    baseAcceleration.setZero();

    motorTorque.resize(subModel.getModel().getNrOfDOFs());
    motorTorque.setZero();

    frictionTorque.resize(subModel.getModel().getNrOfDOFs());
    frictionTorque.setZero();

    jointVelocity.resize(subModel.getModel().getNrOfDOFs());
    jointVelocity.setZero();

    totalTorqueFromContacts.resize(subModel.getModel().getNrOfDOFs());
    totalTorqueFromContacts.setZero();

    torqueFromContact.resize(subModel.getModel().getNrOfDOFs());
    torqueFromContact.setZero();

    wrench.setZero();

    return true;
}

void RDE::SubModelDynamics::setState(const Eigen::Ref<const Eigen::VectorXd> ukfState,
                                     const Eigen::Ref<const Eigen::VectorXd> jointVelocityFullModel,
                                     const Eigen::Ref<const Eigen::VectorXd> motorTorqueFullModel,
                                     const Eigen::Ref<const Eigen::VectorXd> frictionTorqueFullModel,
                                     const System::VariablesHandler& variableHandler)
{
    for (int idx = 0; idx < subModel.getModel().getNrOfDOFs(); idx++)
    {
        jointVelocity(idx) = jointVelocityFullModel(subModel.getJointMapping().at(idx));
        motorTorque(idx) = motorTorqueFullModel(subModel.getJointMapping().at(idx));
        frictionTorque(idx) = frictionTorqueFullModel(subModel.getJointMapping().at(idx));
    }

    for (int idx = 0; idx < subModel.getNrOfFTSensor(); idx++)
    {
        FTMap[subModel.getFTSensor(idx).name] = ukfState.segment(variableHandler.getVariable(subModel.getFTSensor(idx).name).offset,
                                                                 variableHandler.getVariable(subModel.getFTSensor(idx).name).size);
    }

    for (int idx = 0; idx < subModel.getNrOfExternalContact(); idx++)
    {
        extContactMap[subModel.getExternalContact(idx)] = ukfState.segment(variableHandler.getVariable(subModel.getExternalContact(idx)).offset,
                                                                           variableHandler.getVariable(subModel.getExternalContact(idx)).size);
    }
}

bool RDE::SubModelDynamics::update(manif::SE3d::Tangent& fullModelBaseAcceleration,
                                   Eigen::Ref<const Eigen::VectorXd> fullModelJointAcceleration,
                                   Eigen::Ref<Eigen::VectorXd> updatedJointAcceleration)
{
    constexpr auto errorPrefix = "[SubModelDynamics::update]";

    computeTotalTorqueFromContacts();

    if (!kinDynWrapper->getBaseAcceleration(fullModelBaseAcceleration, fullModelJointAcceleration, baseAcceleration))
    {
        log()->error("{} Cannot compute the sub-model base acceleration.", errorPrefix);
        return false;
    }

    if (!kinDynWrapper->forwardDynamics(motorTorque,
                                        frictionTorque,
                                        totalTorqueFromContacts,
                                        baseAcceleration.coeffs(),
                                        updatedJointAcceleration))
    {
        log()->error("{} Cannot compute the inverse dynamics.", errorPrefix);
        return false;
    }

    return true;
}

void RDE::SubModelDynamics::computeTotalTorqueFromContacts()
{
    totalTorqueFromContacts.setZero();

    // Contribution of FT measurements
    for (int idx = 0; idx < subModel.getNrOfFTSensor(); idx++)
    {
        wrench = (int)subModel.getFTSensor(idx).forceDirection * FTMap[subModel.getFTSensor(idx).name].array();

        torqueFromContact = kinDynWrapper->getFTJacobian(subModel.getFTSensor(idx).name).block(0, 6, 6, subModel.getModel().getNrOfDOFs()).transpose() * wrench;

        totalTorqueFromContacts = totalTorqueFromContacts.array() + torqueFromContact.array();
    }

    // Contribution of unknown external contacts
    for (int idx = 0; idx < subModel.getNrOfExternalContact(); idx++)
    {
        torqueFromContact = kinDynWrapper->getExtContactJacobian(subModel.getExternalContact(idx)).block(0, 6, 6, subModel.getModel().getNrOfDOFs()).transpose() * extContactMap[subModel.getExternalContact(idx)];

        totalTorqueFromContacts = totalTorqueFromContacts.array() + torqueFromContact.array();
    }
}
