/**
 * @file SubModel.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// iDyn
#include <iDynTree/SubModel.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/ModelTransformers.h>
#include <iDynTree/SixAxisForceTorqueSensor.h>

// BLF
#include <BipedalLocomotion/TextLogging/Logger.h>

// RDE
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>

using namespace BipedalLocomotion;
namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

bool RDE::SubModel::isValid() const
{
    return (m_model.getNrOfLinks() > 0);
}

bool RDE::SubModelCreator::splitModel(const std::vector<std::string>& ftFrameList,
                                      std::vector<iDynTree::Model>& idynSubModels)
{
    constexpr auto logPrefix = "[SubModelCreator::splitModel]";

    iDynTree::SubModelDecomposition subModelDecomp;
    iDynTree::Traversal fullModelTraversal;

    // Compute the full model traversal
    if (!this->m_model.computeFullTreeTraversal(fullModelTraversal))
    {
        log()->error("{} Unable to compute the full model traversal.", logPrefix);
        return false;
    }

    // Split full model traversal along ft sensors
    if (!subModelDecomp.splitModelAlongJoints(this->m_model, fullModelTraversal, ftFrameList))
    {
        log()->error("{} Unable to split the full model traversal in submodels along the ft "
                          "sensors.",
                          logPrefix);
        return false;
    }

    // Extract subModel objects from traversals
    for (int idx = 0; idx < subModelDecomp.getNrOfSubModels(); idx++)
    {
        const iDynTree::Traversal& subModelTraversal = subModelDecomp.getTraversal(idx);
        iDynTree::Model subModel;

        if (!iDynTree::extractSubModel(this->m_model, subModelTraversal, subModel))
        {
            log()->error("{} Unable to get the Model object associated to the subModel {}.",
                              logPrefix,
                              idx);
            return false;
        }

        idynSubModels.push_back(subModel);
    }

    return true;
}

std::unordered_map<std::string, RDE::FTSensor>
RDE::SubModelCreator::attachFTsToSubModel(const std::vector<RDE::FTSensor>& ftListFromConfig,
                                          iDynTree::Model& idynSubModel)
{
    std::unordered_map<std::string, RDE::FTSensor> ftList;

    for (const auto& ftFromConfig : ftListFromConfig)
    {
        // If the sensor is contained in the model save force direction
        // and parent link in the struct FT.
        // Otherwise, if one of the two attached links is in the model
        // add the ft frame to the model as additional frame and save
        // information on force directiona and parent link

        auto ftIdx = this->m_sensorList.getSensorIndex(iDynTree::SIX_AXIS_FORCE_TORQUE,
                                                       ftFromConfig.frame);

        iDynTree::SixAxisForceTorqueSensor* sensorFTFromModel
            = static_cast<iDynTree::SixAxisForceTorqueSensor*>(
                this->m_sensorList.getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE, ftIdx));

        // Get link the link on which the measure force is applied.
        std::string linkAppliedWrenchName
            = this->m_model.getLinkName(sensorFTFromModel->getAppliedWrenchLink());

        if (idynSubModel.isFrameNameUsed(ftFromConfig.frame))
        {
            RDE::FTSensor ft;
            ft.name = ftFromConfig.name;
            ft.frame = ftFromConfig.frame;

            // Retrieve force direction
            if (idynSubModel.isLinkNameUsed(linkAppliedWrenchName))
            {
                ft.forceDirection = RDE::FTSensor::Direction::Positive;
            } else
            {
                ft.forceDirection = RDE::FTSensor::Direction::Negative;
            }

            ft.frameIndex = idynSubModel.getFrameIndex(ft.frame);

            ftList[ft.name] = std::move(ft);
        } else
        {
            const std::string firstLink = sensorFTFromModel->getFirstLinkName();
            const std::string secondLink = sensorFTFromModel->getSecondLinkName();

            // If one of the two connected links are in the submodel
            if (idynSubModel.isLinkNameUsed(firstLink) || idynSubModel.isLinkNameUsed(secondLink))
            {
                if (idynSubModel.isLinkNameUsed(firstLink))
                {
                    idynSubModel.addAdditionalFrameToLink(
                        firstLink,
                        ftFromConfig.frame,
                        this->m_kinDyn->getRelativeTransform(this->m_kinDyn->model().getLinkIndex(
                                                                 firstLink),
                                                             this->m_kinDyn->model().getFrameIndex(
                                                                 ftFromConfig.frame)));
                } else
                {
                    idynSubModel.addAdditionalFrameToLink(
                        secondLink,
                        ftFromConfig.frame,
                        this->m_kinDyn->getRelativeTransform(this->m_kinDyn->model().getLinkIndex(
                                                                 secondLink),
                                                             this->m_kinDyn->model().getFrameIndex(
                                                                 ftFromConfig.frame)));
                }

                RDE::FTSensor ft;
                ft.name = ftFromConfig.name;
                ft.frame = ftFromConfig.frame;

                ft.frameIndex = idynSubModel.getFrameIndex(ft.frame);

                if (idynSubModel.isLinkNameUsed(linkAppliedWrenchName))
                {
                    ft.forceDirection = RDE::FTSensor::Direction::Positive;
                } else
                {
                    ft.forceDirection = RDE::FTSensor::Direction::Negative;
                }

                ftList[ft.name] = std::move(ft);
            }
        }
    }

    return ftList;
}

std::unordered_map<std::string, RDE::Sensor> RDE::SubModelCreator::attachAccelerometersToSubModel(
    const std::vector<RDE::Sensor>& accListFromConfig, const iDynTree::Model& subModel)
{
    std::unordered_map<std::string, RDE::Sensor> accList;

    for (int idx = 0; idx < accListFromConfig.size(); idx++)
    {
        if (subModel.isFrameNameUsed(accListFromConfig[idx].frame))
        {
            RDE::Sensor acc;
            acc.name = accListFromConfig[idx].name;
            acc.frame = accListFromConfig[idx].frame;
            acc.frameIndex = subModel.getFrameIndex(acc.frame);
            accList[acc.name] = std::move(acc);
        }
    }

    return accList;
}

std::unordered_map<std::string, RDE::Sensor>
RDE::SubModelCreator::attachGyroscopesToSubModel(const std::vector<RDE::Sensor>& gyroListFromConfig,
                                                 const iDynTree::Model& subModel)
{
    std::unordered_map<std::string, RDE::Sensor> gyroList;

    for (int idx = 0; idx < gyroListFromConfig.size(); idx++)
    {
        if (subModel.isFrameNameUsed(gyroListFromConfig[idx].frame))
        {
            RDE::Sensor gyro;
            gyro.name = gyroListFromConfig[idx].name;
            gyro.frame = gyroListFromConfig[idx].frame;
            gyro.frameIndex = subModel.getFrameIndex(gyro.frame);
            gyroList[gyro.name] = std::move(gyro);
        }
    }

    return gyroList;
}

std::unordered_map<std::string, RDE::Sensor> RDE::SubModelCreator::attachExternalContactsToSubModel(
    const std::vector<RDE::Sensor>& contactsFromConfig, const iDynTree::Model& subModel)
{
    std::unordered_map<std::string, RDE::Sensor> contactList;

    for (int idx = 0; idx < contactsFromConfig.size(); idx++)
    {
        if (subModel.isFrameNameUsed(contactsFromConfig[idx].frame))
        {
            RDE::Sensor contact;
            contact.name = contactsFromConfig[idx].name;
            contact.frame = contactsFromConfig[idx].frame;
            contact.frameIndex = subModel.getFrameIndex(contact.frame);
            contactList[contact.name] = std::move(contact);
        }
    }

    return contactList;
}

std::vector<int> RDE::SubModelCreator::createJointMapping(const iDynTree::Model& subModel)
{
    std::vector<int> jointListMapping;

    for (int idx = 0; idx < subModel.getNrOfJoints(); idx++)
    {
        if (subModel.getJoint(idx)->getNrOfDOFs() > 0)
        {
            jointListMapping.push_back(this->m_model.getJointIndex(subModel.getJointName(idx)));
        }
    }
    return jointListMapping;
}

RDE::SubModel
RDE::SubModelCreator::populateSubModel(iDynTree::Model& idynSubModel,
                                       const std::vector<RDE::FTSensor>& ftList,
                                       const std::vector<RDE::Sensor>& accList,
                                       const std::vector<RDE::Sensor>& gyroList,
                                       const std::vector<RDE::Sensor>& externalContacts)
{
    RDE::SubModel subModel;

    subModel.m_ftList = RDE::SubModelCreator::attachFTsToSubModel(ftList, idynSubModel);

    subModel.m_model = idynSubModel.copy();

    subModel.m_accelerometerList
        = RDE::SubModelCreator::attachAccelerometersToSubModel(accList, idynSubModel);

    subModel.m_gyroscopeList
        = RDE::SubModelCreator::attachGyroscopesToSubModel(gyroList, idynSubModel);

    subModel.m_externalContactList
        = RDE::SubModelCreator::attachExternalContactsToSubModel(externalContacts, idynSubModel);

    subModel.m_jointListMapping = RDE::SubModelCreator::createJointMapping(idynSubModel);

    std::string baseLink = idynSubModel.getLinkName(0);

    // The first IMU found in the model is used as base
    int frameIdx = 0;
    bool frameFound = false;
    while(!frameFound && frameIdx < idynSubModel.getNrOfFrames())
    {
        std::string frameName = idynSubModel.getFrameName(frameIdx);

        for (const auto& acc : subModel.m_accelerometerList)
        {
            if (acc.second.frame == frameName)
            {
                subModel.m_imuBaseFrameIndex = frameIdx;
                subModel.m_imuBaseFrameName = frameName;
                frameFound = true;
                break;
            }
        }
        frameIdx++;
    }

    return subModel;
}

bool RDE::SubModelCreator::createSubModels(const std::vector<RDE::FTSensor>& ftSensorList,
                                           const std::vector<RDE::Sensor>& accList,
                                           const std::vector<RDE::Sensor>& gyroList,
                                           const std::vector<RDE::Sensor>& contactList)
{
    constexpr auto logPrefix = "[SubModelCreator::createSubModels]";

    // Split model in submodels
    std::vector<std::string> ftList;
    for (auto idx = 0; idx < ftSensorList.size(); idx++)
    {
        ftList.push_back(ftSensorList[idx].associatedJoint);
    }

    std::vector<iDynTree::Model> idynSubModels;
    if (!this->splitModel(ftList, idynSubModels))
    {
        log()->error("{} Unable to split the model in submodels.", logPrefix);
        return false;
    }

    if (idynSubModels.size() == 0)
    {
        this->m_subModelList.emplace_back(RDE::SubModelCreator::populateSubModel(this->m_model,
                                                                                 ftSensorList,
                                                                                 accList,
                                                                                 gyroList,
                                                                                 contactList));
    } else
    {
        for (int idxSubModel = 0; idxSubModel < idynSubModels.size(); idxSubModel++)
        {
            this->m_subModelList.emplace_back(
                RDE::SubModelCreator::populateSubModel(idynSubModels[idxSubModel],
                                                       ftSensorList,
                                                       accList,
                                                       gyroList,
                                                       contactList));
        }
    }

    return true;
}

bool RDE::SubModelCreator::createSubModels(
    std::weak_ptr<const ParametersHandler::IParametersHandler> parameterHandler)
{
    constexpr auto logPrefix = "[SubModelCreator::getSubModels]";

    auto ptr = parameterHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The handler has to point to an already initialized "
                          "IParametershandler.",
                          logPrefix);
        return false;
    }

    auto populateSensorParameters = [&ptr, logPrefix](const std::string& groupName,
                                                      std::vector<std::string>& names,
                                                      std::vector<std::string>& frames) -> bool {
        auto group = ptr->getGroup(groupName).lock();
        if (group == nullptr)
        {
            log()->error("{} Unable to get the group named '{}'.", logPrefix, groupName);
            return false;
        }

        if (!group->getParameter("names", names))
        {
            log()->error("{} The parameter handler could not find 'names' in the "
                              "configuration file for the group {}.",
                              logPrefix,
                              groupName);
            return false;
        }

        if (!group->getParameter("frames", frames))
        {
            log()->error("{} The parameter handler could not find 'frames' in the "
                              "configuration file for the group {}.",
                              logPrefix,
                              groupName);
            return false;
        }

        return true;
    };

    std::vector<std::string> ftNames, ftFrames, ftAssociatedJoints;
    bool ok = populateSensorParameters("FT", ftNames, ftFrames);

    auto ftGroup = ptr->getGroup("FT").lock();
    if (ftGroup == nullptr)
    {
        log()->error("{} Unable to get the group names 'FT'.", logPrefix);
        return false;
    }
    ok = ok && ftGroup->getParameter("associated_joints", ftAssociatedJoints);

    std::vector<RDE::FTSensor> ftList;
    for (auto idx = 0; idx < ftNames.size(); idx++)
    {
        RDE::FTSensor ft;
        ft.name = ftNames[idx];
        ft.frame = ftFrames[idx];
        ft.associatedJoint = ftAssociatedJoints[idx];
        ftList.push_back(std::move(ft));
    }

    std::vector<std::string> accNames, accFrames;
    ok = ok && populateSensorParameters("ACCELEROMETER", accNames, accFrames);

    std::vector<RDE::Sensor> accList;
    for (auto idx = 0; idx < accNames.size(); idx++)
    {
        RDE::Sensor acc;
        acc.name = accNames[idx];
        acc.frame = accFrames[idx];
        accList.push_back(std::move(acc));
    }

    std::vector<std::string> gyroNames, gyroFrames;
    ok = ok && populateSensorParameters("GYROSCOPE", gyroNames, gyroFrames);

    std::vector<RDE::Sensor> gyroList;
    for (auto idx = 0; idx < gyroNames.size(); idx++)
    {
        RDE::Sensor gyro;
        gyro.name = gyroNames[idx];
        gyro.frame = gyroFrames[idx];
        gyroList.push_back(std::move(gyro));
    }

    std::vector<std::string> contactNames, contactFrames;
    ok = ok
         && populateSensorParameters("EXTERNAL_CONTACT",
                                     contactNames,
                                     contactFrames);

    std::vector<RDE::Sensor> contactList;
    for (auto idx = 0; idx < contactNames.size(); idx++)
    {
        RDE::Sensor contact;
        contact.name = contactNames[idx];
        contact.frame = contactFrames[idx];
        contactList.push_back(std::move(contact));
    }

    ok = ok && RDE::SubModelCreator::createSubModels(ftList, accList, gyroList, contactList);

    return ok;
}

void RDE::SubModelCreator::setModelAndSensors(const iDynTree::Model& model,
                                              const iDynTree::SensorsList& sensors)
{
    m_model = model;

    m_sensorList = sensors;
}

bool RDE::SubModelCreator::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[SubModelCreator::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    this->m_kinDyn = kinDyn;

    return true;
}

std::size_t RDE::SubModelCreator::getNrOfSubModels() const
{
    return this->m_subModelList.size();
}

const std::vector<RDE::SubModel>& RDE::SubModelCreator::getSubModelList() const
{
    return this->m_subModelList;
}

const RDE::SubModel& RDE::SubModelCreator::getSubModel(int index) const
{
    return this->m_subModelList.at(index);
}

const iDynTree::Model& RDE::SubModel::getModel() const
{
    return this->m_model;
}

const std::vector<int>& RDE::SubModel::getJointMapping() const
{
    return this->m_jointListMapping;
}

const std::unordered_map<std::string, RDE::FTSensor>& RDE::SubModel::getFTList() const
{
    return this->m_ftList;
}

const std::unordered_map<std::string, RDE::Sensor>& RDE::SubModel::getAccelerometerList() const
{
    return this->m_accelerometerList;
}

const std::unordered_map<std::string, RDE::Sensor>& RDE::SubModel::getGyroscopeList() const
{
    return this->m_gyroscopeList;
}

const std::unordered_map<std::string, RDE::Sensor>& RDE::SubModel::getExternalContactList() const
{
    return this->m_externalContactList;
}

std::size_t RDE::SubModel::getNrOfFTSensor() const
{
    return m_ftList.size();
}

std::size_t RDE::SubModel::getNrOfAccelerometer() const
{
    return m_accelerometerList.size();
}

std::size_t RDE::SubModel::getNrOfGyroscope() const
{
    return m_gyroscopeList.size();
}

std::size_t RDE::SubModel::getNrOfExternalContact() const
{
    return m_externalContactList.size();
}

const RDE::FTSensor& RDE::SubModel::getFTSensor(const std::string& name)
{
    auto ftIterator = m_ftList.find(name);

    if (ftIterator != m_ftList.end())
    {
        return ftIterator->second;
    }

    log()->error("[SubModel::getFTSensor] Sensor `{}` not found.", name);

    return dummyFT;
}

bool RDE::SubModel::hasFTSensor(const std::string& name) const
{
    return m_ftList.find(name) != m_ftList.end();
}

const RDE::Sensor& RDE::SubModel::getAccelerometer(const std::string& name)
{
    auto accIterator = m_accelerometerList.find(name);

    if (accIterator != m_accelerometerList.end())
    {
        return accIterator->second;
    }

    log()->error("[SubModel::getAccelerometer] Sensor `{}` not found.", name);

    return dummySensor;
}

bool RDE::SubModel::hasAccelerometer(const std::string& name) const
{
    return m_accelerometerList.find(name) != m_accelerometerList.end();
}

const RDE::Sensor& RDE::SubModel::getGyroscope(const std::string& name)
{
    auto gyroIterator = m_gyroscopeList.find(name);

    if (gyroIterator != m_gyroscopeList.end())
    {
        return gyroIterator->second;
    }

    log()->error("[SubModel::getGyroscope] Sensor `{}` not found.", name);

    return dummySensor;
}

bool RDE::SubModel::hasGyroscope(const std::string& name) const
{
    return m_gyroscopeList.find(name) != m_gyroscopeList.end();
}

const RDE::Sensor& RDE::SubModel::getExternalContactIndex(const std::string& name)
{
    auto contactIterator = m_externalContactList.find(name);

    if (contactIterator != m_externalContactList.end())
    {
        return contactIterator->second;
    }

    log()->error("[SubModel::getExternalContactIndex] Contact `{}` not found.", name);

    return dummySensor;
}

const std::string RDE::SubModel::getImuBaseFrameName() const
{
    return m_imuBaseFrameName;
}

int RDE::SubModel::getImuBaseFrameIndex() const
{
    return m_imuBaseFrameIndex;
}
