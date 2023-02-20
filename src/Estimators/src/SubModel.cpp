/**
 * @file SubModel.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// iDyn
#include <iDynTree/Model/SubModel.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/ModelTransformers.h>
#include <iDynTree/Sensors/SixAxisForceTorqueSensor.h>

// BLF
#include <BipedalLocomotion/TextLogging/Logger.h>

// RDE
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>

namespace blf = BipedalLocomotion;
namespace blfEstim = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

bool blfEstim::SubModelCreator::splitModel(const std::vector<std::string>& ftFrameList,
                                           std::vector<iDynTree::Model>& idynSubModels)
{
    constexpr auto logPrefix = "[BipedalLocomotion::RobotDynamicsEstimator::SubModelCreator::"
                               "splitModel]";

    iDynTree::SubModelDecomposition subModelDecomp;
    iDynTree::Traversal fullModelTraversal;

    // Compute the full model traversal
    if (!this->m_model.computeFullTreeTraversal(fullModelTraversal))
    {
        blf::log()->error("{} Unable to compute the full model traversal.", logPrefix);
        return false;
    }

    // Split full model traversal along ft sensors
    if (!subModelDecomp.splitModelAlongJoints(this->m_model, fullModelTraversal, ftFrameList))
    {
        blf::log()->error("{} Unable to split the full model traversal in submodels along the ft "
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
            blf::log()->error("{} Unable to get the Model object associated to the subModel {}.",
                              logPrefix,
                              idx);
            return false;
        }

        idynSubModels.push_back(subModel);
    }

    return true;
}

std::vector<blfEstim::FT>
blfEstim::SubModelCreator::attachFTsToSubModel(iDynTree::Model& idynSubModel)
{
    std::vector<blfEstim::FT> ftList;

    for (int ftIdx = 0; ftIdx < this->m_sensorList.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);
         ftIdx++)
    {
        // If the sensor is contained in the model save force direction
        // and parent link in the struct FT.
        // Otherwise, if one of the two attached links is in the model
        // add the ft frame to the model as additional frame and save
        // information on force directiona and parent link
        iDynTree::SixAxisForceTorqueSensor* sensorFTFromModel
            = static_cast<iDynTree::SixAxisForceTorqueSensor*>(
                this->m_sensorList.getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE, ftIdx));
        std::string ftName = sensorFTFromModel->getName();

        // Get link the link on which the measure force is applied.
        std::string linkAppliedWrenchName
            = this->m_model.getLinkName(sensorFTFromModel->getAppliedWrenchLink());

        if (idynSubModel.getFrameIndex(ftName) >= 0)
        {
            blfEstim::FT ft;
            ft.name = ftName;
            ft.frame = ftName;

            // Retrieve force direction
            if (idynSubModel.isLinkNameUsed(linkAppliedWrenchName))
            {
                ft.forceDirection = blfEstim::FT::Direction::Positive;
            } else
            {
                ft.forceDirection = blfEstim::FT::Direction::Negative;
            }

            ftList.push_back(ft);
        } else
        {
            const std::string firstLink = sensorFTFromModel->getFirstLinkName();
            const std::string secondLink = sensorFTFromModel->getSecondLinkName();

            // If one of the two connected links are in the submodel
            if (idynSubModel.isLinkNameUsed(firstLink) || idynSubModel.isLinkNameUsed(secondLink))
            {
                if (idynSubModel.isLinkNameUsed(firstLink))
                {
                    idynSubModel
                        .addAdditionalFrameToLink(firstLink,
                                                  ftName,
                                                  this->m_kinDyn
                                                      ->getRelativeTransform(this->m_kinDyn->model()
                                                                                 .getLinkIndex(
                                                                                     firstLink),
                                                                             this->m_kinDyn->model()
                                                                                 .getFrameIndex(
                                                                                     ftName)));
                } else
                {
                    idynSubModel
                        .addAdditionalFrameToLink(secondLink,
                                                  ftName,
                                                  this->m_kinDyn
                                                      ->getRelativeTransform(this->m_kinDyn->model()
                                                                                 .getLinkIndex(
                                                                                     secondLink),
                                                                             this->m_kinDyn->model()
                                                                                 .getFrameIndex(
                                                                                     ftName)));
                }

                blfEstim::FT ft;
                ft.name = ftName;
                ft.frame = ftName;

                if (idynSubModel.isLinkNameUsed(linkAppliedWrenchName))
                {
                    ft.forceDirection = blfEstim::FT::Direction::Positive;
                } else
                {
                    ft.forceDirection = blfEstim::FT::Direction::Negative;
                }

                ftList.push_back(std::move(ft));
            }
        }
    }

    return ftList;
}

std::vector<blfEstim::Sensor> blfEstim::SubModelCreator::attachAccelerometersToSubModel(
    const std::vector<blfEstim::Sensor>& accListFromConfig, const iDynTree::Model& subModel)
{
    std::vector<blfEstim::Sensor> accList;

    for (int idx = 0; idx < accListFromConfig.size(); idx++)
    {
        if (subModel.isFrameNameUsed(accListFromConfig[idx].frame))
        {
            blfEstim::Sensor acc;
            acc.name = accListFromConfig[idx].name;
            acc.frame = accListFromConfig[idx].frame;
            accList.push_back(std::move(acc));
        }
    }

    return accList;
}

std::vector<blfEstim::Sensor> blfEstim::SubModelCreator::attachGyroscopesToSubModel(
    const std::vector<blfEstim::Sensor>& gyroListFromConfig, const iDynTree::Model& subModel)
{
    std::vector<blfEstim::Sensor> gyroList;

    for (int idx = 0; idx < gyroListFromConfig.size(); idx++)
    {
        if (subModel.isFrameNameUsed(gyroListFromConfig[idx].frame))
        {
            blfEstim::Sensor gyro;
            gyro.name = gyroListFromConfig[idx].name;
            gyro.frame = gyroListFromConfig[idx].frame;
            gyroList.push_back(std::move(gyro));
        }
    }

    return gyroList;
}

std::vector<std::string> blfEstim::SubModelCreator::attachExternalContactsToSubModel(
    const std::vector<std::string>& contactsFromConfig, const iDynTree::Model& subModel)
{
    std::vector<std::string> contactList;

    for (int idx = 0; idx < contactsFromConfig.size(); idx++)
    {
        if (subModel.isFrameNameUsed(contactsFromConfig[idx]))
        {
            contactList.push_back(contactsFromConfig[idx]);
        }
    }

    return contactList;
}

std::vector<int> blfEstim::SubModelCreator::createJointMapping(const iDynTree::Model& subModel)
{
    std::vector<int> jointListMapping;

    for (int idx = 0; idx < subModel.getNrOfJoints(); idx++)
    {
        jointListMapping.push_back(this->m_model.getJointIndex(subModel.getJointName(idx)));
    }

    return jointListMapping;
}

blfEstim::SubModel
blfEstim::SubModelCreator::populateSubModel(iDynTree::Model& idynSubModel,
                                            const std::vector<blfEstim::Sensor>& accList,
                                            const std::vector<blfEstim::Sensor>& gyroList,
                                            const std::vector<std::string>& externalContacts)
{
    blfEstim::SubModel subModel;

    subModel.m_ftList = blfEstim::SubModelCreator::attachFTsToSubModel(idynSubModel);

    subModel.m_model = idynSubModel.copy();

    subModel.m_accelerometerList
        = blfEstim::SubModelCreator::attachAccelerometersToSubModel(accList, idynSubModel);

    subModel.m_gyroscopeList
        = blfEstim::SubModelCreator::attachGyroscopesToSubModel(gyroList, idynSubModel);

    subModel.m_externalContactList
        = blfEstim::SubModelCreator::attachExternalContactsToSubModel(externalContacts,
                                                                      idynSubModel);

    subModel.m_jointListMapping = blfEstim::SubModelCreator::createJointMapping(idynSubModel);

    return subModel;
}

bool blfEstim::SubModelCreator::createSubModels(const std::vector<blfEstim::Sensor>& ftSensorList,
                                                const std::vector<blfEstim::Sensor>& accList,
                                                const std::vector<blfEstim::Sensor>& gyroList,
                                                const std::vector<std::string>& externalContacts)
{
    constexpr auto logPrefix = "[BipedalLocomotion::RobotDynamicsEstimator::SubModelCreator::"
                               "getSubModels]";

    // Split model in submodels
    std::vector<std::string> ftNameList;
    for (auto idx = 0; idx < ftSensorList.size(); idx++)
    {
        ftNameList.push_back(ftSensorList[idx].frame);
    }

    std::vector<iDynTree::Model> idynSubModels;
    if (!this->splitModel(ftNameList, idynSubModels))
    {
        blf::log()->error("{} Unable to split the model in submodels.", logPrefix);
        return false;
    }

    for (int idxSubModel = 0; idxSubModel < idynSubModels.size(); idxSubModel++)
    {
        this->m_subModelList.emplace_back(
            blfEstim::SubModelCreator::populateSubModel(idynSubModels[idxSubModel],
                                                        accList,
                                                        gyroList,
                                                        externalContacts));
    }

    return true;
}

bool blfEstim::SubModelCreator::createSubModels(
    std::weak_ptr<const blf::ParametersHandler::IParametersHandler> parameterHandler)
{
    constexpr auto logPrefix = "[BipedalLocomotion::Estimators::RobotDynamicsEstimator::"
                               "getSubModels]";

    auto ptr = parameterHandler.lock();
    if (ptr == nullptr)
    {
        blf::log()->error("{} The handler has to point to an already initialized "
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
            blf::log()->error("{} Unable to get the group named '{}'.", logPrefix, groupName);
            return false;
        }

        if (!group->getParameter("names", names))
        {
            blf::log()->error("{} The parameter handler could not find 'names' in the "
                              "configuration file for the group {}.",
                              logPrefix,
                              groupName);
            return false;
        }

        if (!group->getParameter("frames", frames))
        {
            blf::log()->error("{} The parameter handler could not find 'frames' in the "
                              "configuration file for the group {}.",
                              logPrefix,
                              groupName);
            return false;
        }
        return true;
    };

    auto extContactGroup = ptr->getGroup("EXTERNAL_CONTACT").lock();
    if (extContactGroup == nullptr)
    {
        blf::log()->error("{} Unable to get the group named 'EXTERNAL_CONTACT'.", logPrefix);
        return false;
    }

    std::vector<std::string> extContactFrames;
    if (!extContactGroup->getParameter("frames", extContactFrames))
    {
        blf::log()->warn("{} The parameter handler could not find \" frames \" in the "
                         "configuration file.",
                         logPrefix);
    }

    std::vector<std::string> ftNames, ftFrames;
    bool ok = populateSensorParameters("FT", ftNames, ftFrames);

    std::vector<blfEstim::Sensor> ftList;
    for (auto idx = 0; idx < ftNames.size(); idx++)
    {
        blfEstim::Sensor ft;
        ft.name = ftNames[idx];
        ft.frame = ftFrames[idx];
        ftList.push_back(std::move(ft));
    }

    std::vector<std::string> accNames, accFrames;
    ok = ok && populateSensorParameters("ACCELEROMETER", accNames, accFrames);

    std::vector<blfEstim::Sensor> accList;
    for (auto idx = 0; idx < accNames.size(); idx++)
    {
        blfEstim::Sensor acc;
        acc.name = accNames[idx];
        acc.frame = accFrames[idx];
        accList.push_back(std::move(acc));
    }

    std::vector<std::string> gyroNames, gyroFrames;
    ok = ok && populateSensorParameters("GYROSCOPE", gyroNames, gyroFrames);

    std::vector<blfEstim::Sensor> gyroList;
    for (auto idx = 0; idx < gyroNames.size(); idx++)
    {
        blfEstim::Sensor gyro;
        gyro.name = gyroNames[idx];
        gyro.frame = gyroFrames[idx];
        gyroList.push_back(std::move(gyro));
    }

    ok = ok
         && blfEstim::SubModelCreator::createSubModels(ftList, accList, gyroList, extContactFrames);

    return ok;
}

void blfEstim::SubModelCreator::setModel(const iDynTree::Model& model)
{
    this->m_model = model;
}

bool blfEstim::SubModelCreator::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[FeasibleContactWrenchTask::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    this->m_kinDyn = kinDyn;

    return true;
}

void blfEstim::SubModelCreator::setSensorList(const iDynTree::SensorsList& sensorList)
{
    this->m_sensorList = sensorList;
}

const std::vector<blfEstim::SubModel>& blfEstim::SubModelCreator::getSubModelList() const
{
    return this->m_subModelList;
}

const blfEstim::SubModel& blfEstim::SubModelCreator::getSubModel(int index) const
{
    return this->m_subModelList.at(index);
}

const iDynTree::Model& blfEstim::SubModel::getModel() const
{
    return this->m_model;
}

const std::vector<int>& blfEstim::SubModel::getJointMapping() const
{
    return this->m_jointListMapping;
}

const std::vector<blfEstim::FT>& blfEstim::SubModel::getFTList() const
{
    return this->m_ftList;
}

const std::vector<blfEstim::Sensor>& blfEstim::SubModel::getAccelerometerList() const
{
    return this->m_accelerometerList;
}

const std::vector<blfEstim::Sensor>& blfEstim::SubModel::getGyroscopeList() const
{
    return this->m_gyroscopeList;
}

const std::vector<std::string>& blfEstim::SubModel::getExternalContactList() const
{
    return this->m_externalContactList;
}
