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
        log()->info("{} The parameter 'base_link' is not provided. The default one "
                    "will be used {}.",
                    logPrefix,
                    m_baseLink);
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

    if (!mdlLdr.loadReducedModelFromFile(modelFilePath, jointsAndFTs))
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
    if (!contactGroup->getParameter("frames", contactList))
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
        std::string gyroBias = gyro + "_bias";
        m_estimatorOutput.output.gyroscopeBiases[gyroBias] = Eigen::VectorXd(3).setZero(); // GYRO
                                                                                           // BIAS
    }

    return true;
}

bool RobotDynamicsEstimatorDevice::setEstimatorInitialState()
{
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
    auto fixedFrameIdx = m_iDynEstimator.model().getFrameIndex(m_baseLink);
    auto fullBodyUnknowns = iDynTree::LinkUnknownWrenchContacts(m_iDynEstimator.model());
    fullBodyUnknowns.clear();
    fullBodyUnknowns.addNewUnknownFullWrenchInFrameOrigin(m_iDynEstimator.model(), fixedFrameIdx);

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

    if (!m_iDynEstimator
             .updateKinematicsFromFixedBase(sidyn, dsidyn, ddsidyn, fixedFrameIdx, gravity))
    {
        return false;
    }

    if (!m_iDynEstimator.computeExpectedFTSensorsMeasurements(fullBodyUnknowns,
                                                              expectedFT,
                                                              estimatedContacts,
                                                              estimatedTau))
    {
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

    m_kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    if (!m_kinDyn->loadRobotModel(modelLoader.model()))
    {
        log()->error("{} Failed while setting the robot model to the KinDynComputation object.",
                     logPrefix);
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

    if (!generalGroupHandler->getParameter("port_prefix", m_portPrefix))
    {
        log()->info("{} The parameter 'port_prefix' is not provided. The default one "
                    "will be used {}.",
                    logPrefix,
                    m_portPrefix);
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

    return true;
}

bool RobotDynamicsEstimatorDevice::openCommunications()
{
    if (!m_loggerPort.open(m_portPrefix + "/data:o"))
    {
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

    if (!openCommunications())
    {
        log()->error("{} Could not open ports for publishing outputs.", logPrefix);
        return false;
    }

    // run the thread
    m_publishEstimationThread = std::thread([this] { this->publishEstimatorOutput(); });

    start();

    return true;
}

bool RobotDynamicsEstimatorDevice::updateMeasurements()
{
    m_estimatorInput.input.basePose.setIdentity();
    m_estimatorInput.input.baseVelocity.setZero();
    m_estimatorInput.input.baseAcceleration.setZero();

    if (!m_robotSensorBridge->getJointPositions(m_estimatorInput.input.jointPositions))
    {
        return false;
    }

    if (!m_robotSensorBridge->getJointVelocities(m_estimatorInput.input.jointVelocities))
    {
        return false;
    }

    if (!m_robotSensorBridge->getMotorCurrents(m_estimatorInput.input.motorCurrents))
    {
        return false;
    }

    for (auto& [key, value] : m_estimatorInput.input.ftWrenches)
    {
        if (!m_robotSensorBridge
                 ->getSixAxisForceTorqueMeasurement(key, m_estimatorInput.input.ftWrenches[key]))
        {
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
            return false;
        }
    }

    for (auto& [key, value] : m_estimatorInput.input.angularVelocities)
    {
        if (!m_robotSensorBridge->getGyroscopeMeasure(key,
                                                      m_estimatorInput.input.angularVelocities[key]))
        {
            return false;
        }
    }

    m_robotSensorBridge->getJointTorques(m_measuredTauj);

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
        auto& data = m_loggerPort.prepare();
        data.vectors.clear();

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

            data.vectors["ds::estimated"].assign(m_estimatorOutput.output.ds.data(),
                                                 m_estimatorOutput.output.ds.data()
                                                     + m_estimatorOutput.output.ds.size());
            data.vectors["tau_m::estimated"].assign(m_estimatorOutput.output.tau_m.data(),
                                                    m_estimatorOutput.output.tau_m.data()
                                                        + m_estimatorOutput.output.tau_m.size());
            data.vectors["tau_F::estimated"].assign(m_estimatorOutput.output.tau_F.data(),
                                                    m_estimatorOutput.output.tau_F.data()
                                                        + m_estimatorOutput.output.tau_F.size());

            m_estimatedTauj = m_estimatorOutput.output.tau_m - m_estimatorOutput.output.tau_F;
            data.vectors["tau_j::estimated"].assign(m_estimatedTauj.data(),
                                                    m_estimatedTauj.data()
                                                        + m_estimatedTauj.size());
            data.vectors["tau_j::measured"].assign(m_measuredTauj.data(),
                                                   m_measuredTauj.data() + m_measuredTauj.size());

            for (auto& [key, value] : m_estimatorOutput.output.ftWrenches)
            {
                data.vectors["fts::" + key + "::estimated"].assign(value.data(),
                                                                   value.data() + value.size());
            }
            for (auto& [key, value] : m_estimatorOutput.output.contactWrenches)
            {
                data.vectors["contacts::" + key + "::estimated"].assign(value.data(),
                                                                        value.data()
                                                                            + value.size());
            }
            for (auto& [key, value] : m_estimatorOutput.output.ftWrenchesBiases)
            {
                data.vectors["fts_biases::" + key + "::estimated"].assign(value.data(),
                                                                          value.data()
                                                                              + value.size());
            }
            for (auto& [key, value] : m_estimatorOutput.output.accelerometerBiases)
            {
                data.vectors["accelerometer_biases::" + key + "::estimated"]
                    .assign(value.data(), value.data() + value.size());
            }
        }

        {
            std::lock_guard<std::mutex> lockInput(m_estimatorInput.mutex);

            data.vectors["im::measured"].assign(m_estimatorInput.input.motorCurrents.data(),
                                                m_estimatorInput.input.motorCurrents.data()
                                                    + m_estimatorInput.input.motorCurrents.size());
            data.vectors["ds::measured"].assign(m_estimatorInput.input.jointVelocities.data(),
                                                m_estimatorInput.input.jointVelocities.data()
                                                    + m_estimatorInput.input.jointVelocities.size());
            for (auto& [key, value] : m_estimatorInput.input.ftWrenches)
            {
                data.vectors["fts::" + key + "::measured"].assign(value.data(),
                                                                  value.data() + value.size());
            }
            for (auto& [key, value] : m_estimatorInput.input.linearAccelerations)
            {
                data.vectors["accelerometers::" + key + "::measured"].assign(value.data(),
                                                                             value.data()
                                                                                 + value.size());
            }
            for (auto& [key, value] : m_estimatorInput.input.angularVelocities)
            {
                data.vectors["gyroscopes::" + key + "::measured"].assign(value.data(),
                                                                         value.data()
                                                                             + value.size());
            }
        }
        m_loggerPort.write();

        // Publish on WBD ports
        yarp::eigen::toEigen(m_estimatedJointTorquesYARP) = m_estimatedTauj;
        m_remappedVirtualAnalogSensorsInterfaces.ivirtsens->updateVirtualAnalogSensorMeasure(
            m_estimatedJointTorquesYARP);

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
