/**
 * @file RobotDynamicsEstimatorDevice.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/RobotDynamicsEstimatorDevice.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/YarpUtilities/Helper.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/YARPConversions.h>
#include <yarp/eigen/Eigen.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;

RobotDynamicsEstimatorDevice::RobotDynamicsEstimatorDevice(
    double period, yarp::os::ShouldUseSystemClock useSystemClock)
    : yarp::os::PeriodicThread(period, useSystemClock)
{
}

RobotDynamicsEstimatorDevice::RobotDynamicsEstimatorDevice()
    : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No)
{
}

RobotDynamicsEstimatorDevice::~RobotDynamicsEstimatorDevice() = default;

bool RobotDynamicsEstimatorDevice::setupRobotModel(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler,
    iDynTree::ModelLoader& mdlLdr)
{
    constexpr auto logPrefix = "[RobotDynamicsEstimatorDevice::setupRobotModel]";

    auto ptr = paramHandler.lock();

    std::string modelFileName;
    std::vector<std::string> ftJointList;

    if (!ptr->getParameter("base_link", m_baseLink))
    {
        log()->info("{} The parameter 'base_link' is not provided.",
                    logPrefix);
    }

    if (!ptr->getParameter("contact_frame", m_contactFrame))
    {
        log()->info("{} The parameter 'contact_frame' is not provided.",
                    logPrefix);
    }

    if (!ptr->getParameter("base_imu", m_baseIMU))
    {
        log()->info("{} The parameter 'base_imu' is not provided.",
                    logPrefix);
    }

    if (!ptr->getParameter("model_file", modelFileName))
    {
        log()->error("{} Could not find parameter `model_file`.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("joint_list", m_jointNameList))
    {
        log()->error("{} Could not find parameter `joint_list`.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("gear_ratio", m_gearboxRatio))
    {
        log()->error("{} Could not find parameter `gear_ratio`", logPrefix);
        return false;
    }

    std::vector<std::string> fixedJointsList;
    if (!ptr->getParameter("fixed_joint_list_names", fixedJointsList))
    {
        BipedalLocomotion::log()->debug("{} Unable to find the joint list.", logPrefix);
    }

    // check that none of the fixed joints is in the controlled joints
    for (const auto& fixedJoint : fixedJointsList)
    {
        if (std::find(m_jointNameList.begin(), m_jointNameList.end(), fixedJoint)
            != m_jointNameList.end())
        {
            // If found, throw an error
            BipedalLocomotion::log()->error("{} Fixed joint {} is also in the controlled joints "
                                            "list.",
                                            logPrefix,
                                            fixedJoint);
            return false;
        }
    }

    Eigen::VectorXd fixedJointPositions;
    if (!ptr->getParameter("fixed_joint_list_values", fixedJointPositions))
    {
        BipedalLocomotion::log()->debug("{} Unable to find the list of position values for the fixed joints.", logPrefix);
    }

    if (fixedJointPositions.size() != fixedJointsList.size())
    {
        BipedalLocomotion::log()->error("{} The size of the initial joint position is different "
                                        "from the size of the joint list.",
                                        logPrefix);
        return false;
    }

    // create the map of the fixed joints with the corresponding position
    std::unordered_map<std::string, double> fixedJointsMap;

    for (size_t i{}; i < fixedJointsList.size(); i++)
    {
        fixedJointsMap[fixedJointsList[i]] = fixedJointPositions[i] * M_PI / 180.0;
    }

    if (!ptr->getParameter("torque_constant", m_torqueConstant))
    {
        log()->error("{} Could not find parameter `torque_constant`", logPrefix);
        return false;
    }

    auto ftGroup = ptr->getGroup("FT").lock();
    if (ftGroup == nullptr)
    {
        log()->error("{} Group `FT` does not exist.", logPrefix);
        return false;
    }
    if (!ftGroup->getParameter("associated_joints", ftJointList))
    {
        log()->error("{} Could not find parameter `associated_joints` in group `ft`.", logPrefix);
        return false;
    }

    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string modelFilePath{rf.findFileByName(modelFileName)};
    log()->info("{} Loading model from {}", logPrefix, modelFilePath);

    std::vector<std::string> jointsAndFTs;
    jointsAndFTs.insert(jointsAndFTs.begin(), m_jointNameList.begin(), m_jointNameList.end());
    jointsAndFTs.insert(jointsAndFTs.end(), ftJointList.begin(), ftJointList.end());

    if (!mdlLdr.loadReducedModelFromFile(modelFilePath, jointsAndFTs, fixedJointsMap))
    {
        log()->error("{} Could not load robot model", logPrefix);
        return false;
    }

    return true;
}

bool RobotDynamicsEstimatorDevice::createSubModels(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler,
    iDynTree::ModelLoader& modelLoader,
    std::vector<SubModel>& subModelList,
    std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList)
{
    constexpr auto logPrefix = "[RobotDynamicsEstimatorDevice::createSubModels]";

    SubModelCreator subModelCreator;
    subModelCreator.setModelAndSensors(m_kinDyn->model(), modelLoader.sensors());
    if (!subModelCreator.setKinDyn(m_kinDyn))
    {
        log()->error("{} Failed while setting the KinDynComputation object to the SubModelCreator "
                     "object.",
                     logPrefix);
        return false;
    }

    if (!subModelCreator.createSubModels(paramHandler))
    {
        log()->error("{} Failed while creating the list of SubModel objects.", logPrefix);
        return false;
    }
    subModelList = subModelCreator.getSubModelList();

    for (const auto& subModel : subModelList)
    {
        log()->info("{} Submodel {}.", logPrefix, subModel.getModel().toString());
    }

    for (int idx = 0; idx < subModelList.size(); idx++)
    {
        kinDynWrapperList.emplace_back(std::make_shared<KinDynWrapper>());
        if (!kinDynWrapperList[idx]->setModel(subModelList[idx]))
        {
            log()->error("{} Failed while initializing the `KinDynWrapper` object "
                         "associated to the submodel with index `{}`.",
                         logPrefix,
                         idx);
            return false;
        }
    }

    return true;
}

bool RobotDynamicsEstimatorDevice::setupRobotSensorBridge(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto logPrefix = "[RobotDynamicsEstimatorDevice::setupRobotSensorBridge]";

    auto groupSensorBridge = paramHandler.lock()->getGroup("RobotSensorBridge").lock();
    if (groupSensorBridge == nullptr)
    {
        log()->error("{} Missing required group `RobotSensorBridge`.", logPrefix);
        return false;
    }

    m_robotSensorBridge = std::make_unique<YarpSensorBridge>();
    if (!m_robotSensorBridge->initialize(groupSensorBridge))
    {
        log()->error("{} Could not configure RobotSensorBridge.", logPrefix);
        return false;
    }

    return true;
}

bool RobotDynamicsEstimatorDevice::resizeEstimatorInitialState(
    IParametersHandler::weak_ptr modelHandler)
{
    m_estimatorOutput.output.ds.resize(m_kinDyn->model().getNrOfDOFs());
    m_estimatorOutput.output.tau_m.resize(m_kinDyn->model().getNrOfDOFs());
    m_estimatorOutput.output.tau_F.resize(m_kinDyn->model().getNrOfDOFs());
    m_estimatedTauj.resize(m_kinDyn->model().getNrOfDOFs());
    m_measuredTauj.resize(m_kinDyn->model().getNrOfDOFs());
    m_estimatedJointTorquesYARP.resize(m_kinDyn->model().getNrOfDOFs());

    std::vector<std::string> ftList;
    auto ftGroup = modelHandler.lock()->getGroup("FT").lock();
    if (!ftGroup->getParameter("names", ftList))
    {
        return false;
    }

    for (const auto& ft : ftList)
    {
        m_estimatorOutput.output.ftWrenches[ft] = Eigen::VectorXd::Zero(6);

        std::string ftBias = ft + "_bias";
        m_estimatorOutput.output.ftWrenchesBiases[ftBias] = Eigen::VectorXd(6).setZero(); // FT bias
    }

    std::vector<std::string> contactList;
    auto contactGroup = modelHandler.lock()->getGroup("EXTERNAL_CONTACT").lock();
    if (!contactGroup->getParameter("names", contactList))
    {
        return false;
    }
    for (const auto& contact : contactList)
    {
        m_estimatorOutput.output.contactWrenches[contact] = Eigen::VectorXd::Zero(6);
    }

    std::vector<std::string> accList;
    auto accGroup = modelHandler.lock()->getGroup("ACCELEROMETER").lock();
    if (!accGroup->getParameter("names", accList))
    {
        return false;
    }
    for (auto acc : accList)
    {
        m_estimatorOutput.output.linearAccelerations[acc] = Eigen::VectorXd::Zero(3); // ACC
    }
    for (auto acc : accList)
    {
        std::string accBias = acc + "_bias";
        m_estimatorOutput.output.accelerometerBiases[accBias] = Eigen::VectorXd::Zero(3); // ACC
                                                                                          // BIAS
    }

    std::vector<std::string> gyroList;
    auto gyroGroup = modelHandler.lock()->getGroup("GYROSCOPE").lock();
    if (!gyroGroup->getParameter("names", gyroList))
    {
        return false;
    }
    for (auto gyro : gyroList)
    {
        m_estimatorOutput.output.angularVelocities[gyro] = Eigen::VectorXd(3).setZero(); // GYRO
    }
    for (auto gyro : gyroList)
    {
        std::string gyroBias = gyro + "_bias";
        m_estimatorOutput.output.gyroscopeBiases[gyroBias] = Eigen::VectorXd(3).setZero(); // GYRO
                                                                                           // BIAS
    }

    return true;
}

bool RobotDynamicsEstimatorDevice::setEstimatorInitialState()
{
    // Set logPrefix
    const std::string logPrefix = "[RobotDynamicsEstimatorDevice::setEstimatorInitialState]";

    iDynTree::Model model = m_kinDyn->model();
    if (!model.isValid())
    {
        log()->error("{} The model is not valid.", logPrefix);
        return false;
    }

    Eigen::VectorXd s(m_kinDyn->model().getNrOfDOFs());
    Eigen::VectorXd dds(m_kinDyn->model().getNrOfDOFs());

    if (!m_robotSensorBridge->getJointPositions(s))
    {
        return false;
    }
    if (!m_robotSensorBridge->getJointVelocities(m_estimatorOutput.output.ds))
    {
        return false;
    }
    if (!m_robotSensorBridge->getJointAccelerations(dds))
    {
        return false;
    }

    m_estimatorOutput.output.tau_F.setZero();

    // Set contact information and specify the unknown wrench on the base link
    auto baseFrameIdx = m_iDynEstimator.model().getFrameIndex(m_baseLink);
    auto fullBodyUnknowns = iDynTree::LinkUnknownWrenchContacts(m_iDynEstimator.model());
    fullBodyUnknowns.clear();
    auto contactLinkIdx = m_iDynEstimator.model().getFrameIndex(m_contactFrame);
    fullBodyUnknowns.addNewUnknownFullWrenchInFrameOrigin(m_iDynEstimator.model(), contactLinkIdx);

    // Initialize variables for estimation
    auto expectedFT = iDynTree::SensorsMeasurements(m_iDynEstimator.sensors());
    auto estimatedContacts = iDynTree::LinkContactWrenches(m_iDynEstimator.model());
    auto estimatedTau = iDynTree::JointDOFsDoubleArray(m_iDynEstimator.model());

    iDynTree::Vector3 gravity;
    gravity.zero();
    gravity.setVal(2, -BipedalLocomotion::Math::StandardAccelerationOfGravitation);

    auto sidyn = iDynTree::JointPosDoubleArray(m_iDynEstimator.model());
    auto dsidyn = iDynTree::JointDOFsDoubleArray(m_iDynEstimator.model());
    auto ddsidyn = iDynTree::JointDOFsDoubleArray(m_iDynEstimator.model());

    for (int index = 0; index < sidyn.size(); index++)
    {
        sidyn.setVal(index, s[index]);
        dsidyn.setVal(index, m_estimatorOutput.output.ds[index]);
        ddsidyn.setVal(index, dds[index]);
    }

    Eigen::Vector3d baseIMUAcceleration;
    if (!m_robotSensorBridge->getLinearAccelerometerMeasurement(m_baseIMU, baseIMUAcceleration))
    {
        log()->error("{} Could not get the accelerometer measurement for `{}`.",
                     logPrefix,
                     m_baseLink);
        return false;
    }
    Eigen::Vector3d baseIMUVelocity;
    if (!m_robotSensorBridge->getGyroscopeMeasure(m_baseIMU, baseIMUVelocity))
    {
        log()->error("{} Could not get the gyroscope measurement for `{}`.", logPrefix, m_baseLink);
        return false;
    }

    manif::SE3d baseHimu; // Transform from the base frame to the imu frame.
    // Get transform matrix from imu to base
    auto imuIdx = model.getFrameIndex(m_baseIMU);
    baseHimu = Conversions::toManifPose(model.getFrameTransform(imuIdx));

    manif::SE3Tangentd baseVelocity;
    baseVelocity.setZero();
    baseVelocity.coeffs().tail(3).noalias() = baseHimu.rotation() * baseIMUVelocity;

    Eigen::Vector3d bOmegaIB;
    bOmegaIB = baseVelocity.coeffs().tail(3);

    const Eigen::Vector3d baseProperAcceleration
        = baseHimu.rotation() * baseIMUAcceleration
          - bOmegaIB.cross(bOmegaIB.cross(baseHimu.translation()));

    const Eigen::Vector3d baseLinearVelocity = baseVelocity.coeffs().head(3);
    const Eigen::Vector3d baseAngularVelocity = baseVelocity.coeffs().tail(3);

    if (!m_iDynEstimator.updateKinematicsFromFloatingBase(sidyn,
                                                          dsidyn,
                                                          ddsidyn,
                                                          baseFrameIdx,
                                                          iDynTree::make_span(
                                                              baseProperAcceleration),
                                                          iDynTree::make_span(baseLinearVelocity),
                                                          iDynTree::make_span(baseAngularVelocity)))
    {
        log()->error("{} Failed while updating the kinematics from the floating base.", logPrefix);
        return false;
    }

    if (!m_iDynEstimator.computeExpectedFTSensorsMeasurements(fullBodyUnknowns,
                                                              expectedFT,
                                                              estimatedContacts,
                                                              estimatedTau))
    {
        log()->error("{} Failed while computing the expected FT sensor measurements.", logPrefix);
        return false;
    }

    std::unordered_map<std::string, Eigen::VectorXd> ftFromModel;
    auto ftWrench = iDynTree::Wrench();
    for (auto& [key, value] : m_estimatorOutput.output.ftWrenches)
    {
        auto ftIndex
            = m_iDynEstimator.sensors().getSensorIndex(iDynTree::SIX_AXIS_FORCE_TORQUE, key);

        if (ftIndex == iDynTree::FRAME_INVALID_INDEX)
        {
            return false;
        }

        if (!expectedFT.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE, ftIndex, ftWrench))
        {
            return false;
        }
        ftFromModel[key] = iDynTree::toEigen(ftWrench);

        if (!m_robotSensorBridge
                 ->getSixAxisForceTorqueMeasurement(key, m_estimatorOutput.output.ftWrenches[key]))
        {
            return false;
        }

        m_ftOffset[key] = m_estimatorOutput.output.ftWrenches[key] - ftFromModel[key];
        m_estimatorOutput.output.ftWrenches[key] = ftFromModel[key];
        std::string ftBias = key + "_bias";
        m_estimatorOutput.output.ftWrenchesBiases[ftBias].setZero(); // FT bias
    }

    if (!m_robotSensorBridge->getMotorCurrents(m_estimatorOutput.output.tau_m))
    {
        return false;
    }
    Eigen::VectorXd temp(m_gearboxRatio.size());
    temp = m_gearboxRatio.array() * m_torqueConstant.array();
    m_estimatorOutput.output.tau_m = m_estimatorOutput.output.tau_m.array() * temp.array();

    // set estimator initial state for accelerometers
    for (auto& [key, value] : m_estimatorOutput.output.linearAccelerations)
    {
        if (!m_robotSensorBridge->getLinearAccelerometerMeasurement(key, value))
        {
            return false;
        }
        std::string accBias = key + "_bias";
        m_estimatorOutput.output.accelerometerBiases[accBias].setZero(); // ACC BIAS
    }

    // set estimator initial state for gyroscopes
    for (auto& [key, value] : m_estimatorOutput.output.angularVelocities)
    {
        if (!m_robotSensorBridge->getGyroscopeMeasure(key, value))
        {
            return false;
        }
        std::string gyroBias = key + "_bias";
        m_estimatorOutput.output.gyroscopeBiases[gyroBias].setZero(); // GYRO BIAS
    }

    // set estimator initial state for contact wrenches
    for (auto& [key, value] : m_estimatorOutput.output.contactWrenches)
    {
        m_estimatorOutput.output.contactWrenches[key].setZero();
    }

    if (!m_estimator->setInitialState(m_estimatorOutput.output))
    {
        return false;
    }

    return true;
}

bool RobotDynamicsEstimatorDevice::resizeEstimatorMeasurement(
    IParametersHandler::weak_ptr modelHandler)
{
    m_estimatorInput.input.jointPositions.resize(m_kinDyn->model().getNrOfDOFs());
    m_estimatorInput.input.jointVelocities.resize(m_kinDyn->model().getNrOfDOFs());
    m_estimatorInput.input.motorCurrents.resize(m_kinDyn->model().getNrOfDOFs());
    m_estimatorInput.input.frictionTorques.resize(m_kinDyn->model().getNrOfDOFs());

    std::vector<std::string> ftList;
    auto ftGroup = modelHandler.lock()->getGroup("FT").lock();
    if (!ftGroup->getParameter("names", ftList))
    {
        return false;
    }

    for (const auto& ft : ftList)
    {
        m_estimatorInput.input.ftWrenches[ft] = Eigen::VectorXd::Zero(6);
    }

    std::vector<std::string> accList;
    auto accGroup = modelHandler.lock()->getGroup("ACCELEROMETER").lock();
    if (!accGroup->getParameter("names", accList))
    {
        return false;
    }
    for (auto acc : accList)
    {
        m_estimatorInput.input.linearAccelerations[acc] = Eigen::VectorXd::Zero(3); // ACC BIAS
    }

    std::vector<std::string> gyroList;
    auto gyroGroup = modelHandler.lock()->getGroup("GYROSCOPE").lock();
    if (!gyroGroup->getParameter("names", gyroList))
    {
        return false;
    }
    for (auto gyro : gyroList)
    {
        m_estimatorInput.input.angularVelocities[gyro] = Eigen::VectorXd(3).setZero(); // GYRO BIAS
    }

    return true;
}

bool RobotDynamicsEstimatorDevice::setupRobotDynamicsEstimator(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto logPrefix = "[RobotDynamicsEstimatorDevice::setupRobotDynamicsEstimator]";

    auto modelGroupHandler = paramHandler.lock()->getGroup("MODEL").lock();
    if (modelGroupHandler == nullptr)
    {
        log()->error("{} Group `MODEL` not found in configuration.", logPrefix);
        return false;
    }

    iDynTree::ModelLoader modelLoader;
    if (!setupRobotModel(modelGroupHandler, modelLoader))
    {
        log()->error("{} Failed while setting the robot model.", logPrefix);
        return false;
    }

    // Create kindyn computation object
    m_kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    if (!m_kinDyn->loadRobotModel(modelLoader.model()))
    {
        log()->error("{} Failed while setting the robot model to the KinDynComputation object.",
                     logPrefix);
        return false;
    }

    auto model = m_kinDyn->model();
    if (!model.isValid())
    {
        log()->error("{} The model is not valid.", logPrefix);
        return false;
    }

    if (!m_kinDyn->setFrameVelocityRepresentation(iDynTree::BODY_FIXED_REPRESENTATION))
    {
        log()->error("{} Failed while setting the frame velocity representation.", logPrefix);
        return false;
    }

    if (!m_iDynEstimator.setModelAndSensors(m_kinDyn->model(), modelLoader.sensors()))
    {
        log()->error("{} Impossible to create ExtWrenchesAndJointTorquesEstimator.", logPrefix);
        return false;
    }

    std::vector<SubModel> subModelList;
    std::vector<std::shared_ptr<KinDynWrapper>> kinDynWrapperList;
    if (!createSubModels(modelGroupHandler, modelLoader, subModelList, kinDynWrapperList))
    {
        log()->error("{} Failed while creating the objects needed to handle the sub-models.",
                     logPrefix);
        return false;
    }

    m_estimator = BipedalLocomotion::Estimators::RobotDynamicsEstimator::RobotDynamicsEstimator::
        build(paramHandler, m_kinDyn, subModelList, kinDynWrapperList);
    if (m_estimator == nullptr)
    {
        log()->error("{} Could not create the RobotDynamicsEstimator object.", logPrefix);
        return false;
    }

    return true;
}

bool RobotDynamicsEstimatorDevice::configureVectorsCollectionServer()
{
    bool ok = true;
    ok = ok && m_vectorsCollectionServer.populateMetadata("ds::estimated", m_jointNameList);
    ok = ok && m_vectorsCollectionServer.populateMetadata("ds::measured", m_jointNameList);
    ok = ok && m_vectorsCollectionServer.populateMetadata("tau_m::estimated", m_jointNameList);
    ok = ok && m_vectorsCollectionServer.populateMetadata("tau_F::estimated", m_jointNameList);
    ok = ok && m_vectorsCollectionServer.populateMetadata("tau_F::measured", m_jointNameList);
    ok = ok && m_vectorsCollectionServer.populateMetadata("tau_j::estimated", m_jointNameList);
    ok = ok && m_vectorsCollectionServer.populateMetadata("tau_j::measured", m_jointNameList);
    for (auto& [key, value] : m_estimatorOutput.output.ftWrenches)
    {
        ok = ok
             && m_vectorsCollectionServer.populateMetadata("fts::" + key + "::estimated",
                                                           ftElementNames);
        ok = ok
             && m_vectorsCollectionServer.populateMetadata("fts::" + key + "::measured",
                                                           ftElementNames);
    }
    for (auto& [key, value] : m_estimatorOutput.output.contactWrenches)
    {
        ok = ok
             && m_vectorsCollectionServer.populateMetadata("contacts::" + key + "::estimated",
                                                           ftElementNames);
    }
    for (auto& [key, value] : m_estimatorOutput.output.linearAccelerations)
    {
        ok = ok
             && m_vectorsCollectionServer.populateMetadata("accelerometers::" + key + "::estimated",
                                                           accelerometerElementNames);
        ok = ok
             && m_vectorsCollectionServer.populateMetadata("accelerometers::" + key + "::measured",
                                                           accelerometerElementNames);
    }
    for (auto& [key, value] : m_estimatorOutput.output.angularVelocities)
    {
        ok = ok
             && m_vectorsCollectionServer.populateMetadata("gyroscopes::" + key + "::estimated",
                                                           gyroElementNames);
        ok = ok
             && m_vectorsCollectionServer.populateMetadata("gyroscopes::" + key + "::measured",
                                                           gyroElementNames);
    }
    ok = ok && m_vectorsCollectionServer.finalizeMetadata();

    return ok;
}

bool RobotDynamicsEstimatorDevice::open(yarp::os::Searchable& config)
{
    constexpr auto logPrefix = "[RobotDynamicsEstimatorDevice::open]";
    auto params = std::make_shared<ParametersHandler::YarpImplementation>(config);

    auto generalGroupHandler = params->getGroup("GENERAL").lock();
    if (generalGroupHandler == nullptr)
    {
        log()->error("{} Group `GENERAL` not found in configuration.", logPrefix);
        return false;
    }
    double devicePeriod{0.01};
    if (generalGroupHandler->getParameter("sampling_time", devicePeriod))
    {
        this->setPeriod(devicePeriod);
    } else
    {
        generalGroupHandler->setParameter("sampling_time", devicePeriod);
        if (!params->setGroup("GENERAL", generalGroupHandler))
        {
            log()->error("{} Could not set group `GENERAL`.", logPrefix);
            return false;
        }
    }

    if (!generalGroupHandler->getParameter("robot", m_robot))
    {
        log()->info("{} The parameter 'robot' is not provided. The default one "
                    "will be used {}.",
                    logPrefix,
                    m_robot);
    }

    if (!setupRobotSensorBridge(params))
    {
        log()->error("{} Could not setup the RobotSensorBridge.", logPrefix);
        return false;
    }

    if (!setupRobotDynamicsEstimator(params))
    {
        log()->error("{} Could not configure the estimator.", logPrefix);
        return false;
    }

    auto modelGroupHandler = params->getGroup("MODEL").lock();
    if (modelGroupHandler == nullptr)
    {
        log()->error("{} Group `MODEL` not found in configuration.", logPrefix);
        return false;
    }
    if (!resizeEstimatorInitialState(modelGroupHandler))
    {
        log()->error("{} Could not resize the RobotDynamicsEstimator state.", logPrefix);
        return false;
    }
    if (!resizeEstimatorMeasurement(modelGroupHandler))
    {
        log()->error("{} Could not resize the RobotDynamicsEstimator input.", logPrefix);
        return false;
    }

    if (!openRemapperVirtualSensors())
    {
        log()->error("{} Could not open virtual analog sensors remapper.", logPrefix);
        return false;
    }

    if (!m_vectorsCollectionServer.initialize(generalGroupHandler))
    {
        log()->error("{} Unable to configure the server.", logPrefix);
        return false;
    }

    if (!configureVectorsCollectionServer())
    {
        log()->error("{} Unable to configure the server.", logPrefix);
        return false;
    }

    return true;
}

bool RobotDynamicsEstimatorDevice::openRemapperVirtualSensors()
{
    constexpr auto logPrefix = "[RobotDynamicsEstimatorDevice::openRemapperVirtualSensors]";
    // Pass to the remapper just the relevant parameters (axesList)
    yarp::os::Property propRemapper;
    propRemapper.put("device", "virtualAnalogRemapper");

    propRemapper.addGroup("axesNames");
    yarp::os::Bottle& bot = propRemapper.findGroup("axesNames").addList();
    for (size_t i = 0; i < m_jointNameList.size(); i++)
    {
        bot.addString(m_jointNameList[i].c_str());
    }

    if (!m_remappedVirtualAnalogSensors.open(propRemapper))
    {
        log()->error("{} Error opening the virtual analog sensor port", logPrefix);
        return false;
    }

    // View relevant interfaces for the remappedVirtualAnalogSensors
    bool ok
        = m_remappedVirtualAnalogSensors.view(m_remappedVirtualAnalogSensorsInterfaces.ivirtsens);
    ok = ok
         && m_remappedVirtualAnalogSensors.view(m_remappedVirtualAnalogSensorsInterfaces.multwrap);

    if (!ok)
    {
        log()->error("{} Could not find the necessary interfaces in remappedControlBoard",
                     logPrefix);
        return ok;
    }

    return true;
}

bool RobotDynamicsEstimatorDevice::attachAll(const yarp::dev::PolyDriverList& poly)
{
    constexpr auto logPrefix = "[RobotDynamicsEstimatorDevice::attachAll]";

    if (!m_robotSensorBridge->setDriversList(poly))
    {
        log()->error("{} Failed to attach to devices through RobotSensorBridge.", logPrefix);
        return false;
    }

    if (!m_remappedVirtualAnalogSensorsInterfaces.multwrap->attachAll(poly))
    {
        log()->error("{} Could not attach virtual sensors.", logPrefix);
        return false;
    }

    // run the thread
    m_publishEstimationThread = std::thread([this] { this->publishEstimatorOutput(); });

    start();

    return true;
}

bool RobotDynamicsEstimatorDevice::updateMeasurements()
{
    // Create log prefix
    constexpr auto logPrefix = "[RobotDynamicsEstimatorDevice::updateMeasurements]";

    if (!m_robotSensorBridge->getJointPositions(m_estimatorInput.input.jointPositions))
    {
        log()->error("{} Could not get joint positions.", logPrefix);
        return false;
    }

    if (!m_robotSensorBridge->getJointVelocities(m_estimatorInput.input.jointVelocities))
    {
        log()->error("{} Could not get joint velocities.", logPrefix);
        return false;
    }

    if (!m_robotSensorBridge->getMotorCurrents(m_estimatorInput.input.motorCurrents))
    {
        log()->error("{} Could not get motor currents.", logPrefix);
        return false;
    }

    if (!m_robotSensorBridge->getMotorAccelerations(m_estimatorInput.input.frictionTorques))
    {
        log()->error("{} Could not get motor accelerations.", logPrefix);
        return false;
    }
    m_estimatorInput.input.frictionTorques = m_estimatorInput.input.frictionTorques.array();

    for (auto& [key, value] : m_estimatorInput.input.ftWrenches)
    {
        if (!m_robotSensorBridge
                 ->getSixAxisForceTorqueMeasurement(key, m_estimatorInput.input.ftWrenches[key]))
        {
            log()->error("{} Could not get the FT measurement for `{}`.", logPrefix, key);
            return false;
        }
        m_estimatorInput.input.ftWrenches[key] -= m_ftOffset[key];
    }

    for (auto& [key, value] : m_estimatorInput.input.linearAccelerations)
    {
        if (!m_robotSensorBridge->getLinearAccelerometerMeasurement(key,
                                                                    m_estimatorInput.input
                                                                        .linearAccelerations[key]))
        {
            log()->error("{} Could not get the accelerometer measurement for `{}`.",
                         logPrefix,
                         key);
            return false;
        }
    }

    for (auto& [key, value] : m_estimatorInput.input.angularVelocities)
    {
        if (!m_robotSensorBridge->getGyroscopeMeasure(key,
                                                      m_estimatorInput.input.angularVelocities[key]))
        {
            log()->error("{} Could not get the gyroscope measurement for `{}`.", logPrefix, key);
            return false;
        }
    }

    return true;
}

void RobotDynamicsEstimatorDevice::publishEstimatorOutput()
{
    constexpr auto logPrefix = "[RobotDynamicsEstimatorDevice::publishEstimatorOutput]";

    auto time = BipedalLocomotion::clock().now();
    auto oldTime = time;
    auto wakeUpTime = time;
    const auto publishOutputPeriod = std::chrono::duration<double>(0.01);

    m_estimatorIsRunning = true;

    while (m_estimatorIsRunning)
    {
        m_vectorsCollectionServer.prepareData(); // required to prepare the data to be sent
        m_vectorsCollectionServer.clearData(); // optional see the documentation

        // detect if a clock has been reset
        oldTime = time;
        time = BipedalLocomotion::clock().now();
        // if the current time is lower than old time, the timer has been reset.
        if ((time - oldTime).count() < 1e-12)
        {
            wakeUpTime = time;
        }
        wakeUpTime = std::chrono::duration_cast<std::chrono::nanoseconds>(wakeUpTime
                                                                          + publishOutputPeriod);

        {
            std::lock_guard<std::mutex> lockOutput(m_estimatorOutput.mutex);

            m_vectorsCollectionServer.populateData("ds::estimated", m_estimatorOutput.output.ds);

            m_vectorsCollectionServer.populateData("tau_m::estimated",
                                                   m_estimatorOutput.output.tau_m);

            m_vectorsCollectionServer.populateData("tau_F::estimated",
                                                   m_estimatorOutput.output.tau_F);

            m_estimatedTauj = m_estimatorOutput.output.tau_m - m_estimatorOutput.output.tau_F;
            m_vectorsCollectionServer.populateData("tau_j::estimated", m_estimatedTauj);

            for (auto& [key, value] : m_estimatorOutput.output.ftWrenches)
            {
                m_vectorsCollectionServer.populateData("fts::" + key + "::estimated", value);
            }

            for (auto& [key, value] : m_estimatorOutput.output.contactWrenches)
            {
                m_vectorsCollectionServer.populateData("contacts::" + key + "::estimated", value);
            }

            for (auto& [key, value] : m_estimatorOutput.output.linearAccelerations)
            {
                m_vectorsCollectionServer.populateData("accelerometers::" + key + "::estimated",
                                                       value);
            }

            for (auto& [key, value] : m_estimatorOutput.output.angularVelocities)
            {
                m_vectorsCollectionServer.populateData("gyroscopes::" + key + "::estimated", value);
            }

            m_vectorsCollectionServer.sendData();
        }

        // Publish on WBD ports
        yarp::eigen::toEigen(m_estimatedJointTorquesYARP) = m_estimatedTauj;

        if (!m_remappedVirtualAnalogSensorsInterfaces.ivirtsens->updateVirtualAnalogSensorMeasure(
                m_estimatedJointTorquesYARP))
        {
            log()->error("{} Could not update the virtual analog sensor measure.", logPrefix);
        }

        // release the CPU
        BipedalLocomotion::clock().yield();

        // sleep
        BipedalLocomotion::clock().sleepUntil(wakeUpTime);
    }
}

void RobotDynamicsEstimatorDevice::run()
{
    constexpr auto logPrefix = "[RobotDynamicsEstimatorDevice::run]";

    // advance sensor bridge
    if (!m_robotSensorBridge->advance())
    {
        log()->warn("{} Advance Sensor bridge failed.", logPrefix);
        return;
    }

    if (m_isFirstRun)
    {
        if (!setEstimatorInitialState())
        {
            log()->error("{} Could not set estimator initial state", logPrefix);
            detachAll();
            close();
            return;
        }
        m_isFirstRun = false;
    }

    // update estimator measurements
    {
        std::lock_guard<std::mutex> lockInput(m_estimatorInput.mutex);
        if (!updateMeasurements())
        {
            log()->error("{} Measurement updates failed.", logPrefix);
            return;
        }

        if (!m_estimator->setInput(m_estimatorInput.input))
        {
            log()->error("{} Could not set estimator input.", logPrefix);
            return;
        }
    }

    if (!m_estimator->advance())
    {
        log()->warn("{} Advance RobotDynamicsEstimator failed.", logPrefix);
        return;
    }

    {
        std::lock_guard<std::mutex> lockOutput(m_estimatorOutput.mutex);
        m_estimatorOutput.output = m_estimator->getOutput();
    }

    return;
}

bool RobotDynamicsEstimatorDevice::detachAll()
{
    if (isRunning())
    {
        stop();
    }

    m_remappedVirtualAnalogSensorsInterfaces.multwrap->detachAll();

    return true;
}

bool RobotDynamicsEstimatorDevice::close()
{
    m_estimatorIsRunning = false;
    if (m_publishEstimationThread.joinable())
    {
        m_publishEstimationThread.join();
    }

    m_remappedVirtualAnalogSensors.close();

    if (!m_loggerPort.isClosed())
    {
        m_loggerPort.close();
    }

    return true;
}
