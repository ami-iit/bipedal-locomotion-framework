/**
 * @file FloatingBaseEstimatorDevice.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/FloatingBaseEstimatorDevice.h>
#include <BipedalLocomotion/YarpUtilities/Helper.h>

#include <BipedalLocomotion/Math/Wrench.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <yarp/os/LogStream.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::ParametersHandler;

template<typename BaseEstimator, typename ContactDetector>
bool updateKIFWrapper(KinematicInertialFilterWrapper<BaseEstimator, ContactDetector>* kifWrapper,
                      const ProprioceptiveInput& in,
                      KinematicInertialFilterOutput& out)
{
    bool ok{true};
    ok = ok && kifWrapper->setInput(in);
    ok = ok && kifWrapper->advance();
    out = kifWrapper->getOutput();
    return ok;
}

template<typename BaseEstimator, typename ContactDetector>
bool initializeKIFWrapper(KinematicInertialFilterWrapper<BaseEstimator, ContactDetector>* kifWrapper,
                          IParametersHandler::shared_ptr parameterHandler,
                          std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                          const std::string& type)
{
    const std::string printPrefix{"[BipedalLocomotion::FloatingBaseEstimatorDevice]"};
    if (!kifWrapper->initialize(parameterHandler, kinDyn))
    {
        yError() <<  printPrefix << "[setupBaseEstimator] Could not configure "
                                <<  type << " wrapper.";
        return false;
    }

    return true;
}

FloatingBaseEstimatorDevice::FloatingBaseEstimatorDevice(double period,
                                                         yarp::os::ShouldUseSystemClock useSystemClock)
        : yarp::os::PeriodicThread(period, useSystemClock)
{
}

FloatingBaseEstimatorDevice::FloatingBaseEstimatorDevice()
        : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No)
{
}

FloatingBaseEstimatorDevice::~FloatingBaseEstimatorDevice()
{
}

bool FloatingBaseEstimatorDevice::open(yarp::os::Searchable& config)
{
    YarpUtilities::getElementFromSearchable(config, "robot", m_robot);
    if (!YarpUtilities::getElementFromSearchable(config, "robot_model_uses_front_rear_foot_fts", m_robotModelUsesFrontRearFootFTs))
    {
        yError() << m_printPrefix << "[open] Missing required parameter \"robot_model_uses_front_rear_foot_fts\"";
        return false;
    }

    YarpUtilities::getElementFromSearchable(config, "port_prefix", m_portPrefix);
    YarpUtilities::getElementFromSearchable(config, "base_link_imu", m_baseLinkImuName);

    YarpUtilities::getVectorFromSearchable(config, "left_foot_wrenches", m_leftFootWrenchNames);
    YarpUtilities::getVectorFromSearchable(config, "right_foot_wrenches", m_rightFootWrenchNames);

    if (!m_robotModelUsesFrontRearFootFTs)
    {
        // if robot model is iCubGenova04, expect only one wrench per foot
        if (m_leftFootWrenchNames.size() != 1 || m_rightFootWrenchNames.size() != 1)
        {
            yError() << m_printPrefix << "[open] Expecting only one wrench per foot.";
            return false;
        }
    }
    else if (m_robotModelUsesFrontRearFootFTs)
    {
        // if robot model is iCubGenova09, expect front and rear foot cartesian wrenches
        if (m_leftFootWrenchNames.size() != 2 || m_rightFootWrenchNames.size() != 2)
        {
            yError() << m_printPrefix << "[open] Expecting two wrenches per foot.";
            return false;
        }
    }

    std::string estimatorType;
    if (!YarpUtilities::getElementFromSearchable(config, "estimator_type", estimatorType))
    {
        yError() << m_printPrefix << "[open] Missing required parameter \"estimator_type\"";
        return false;
    }

    if (m_supportedEstimatorLookup.find(estimatorType) == m_supportedEstimatorLookup.end())
    {
        yError() << m_printPrefix << "[open] Specified \"estimator_type\" parameter not in the list of supported estimators."
                 << " Currently supported Base Estimators: LeggedOdometry and InvEKF.";
        return false;
    }
    m_estimatorType = m_supportedEstimatorLookup.at(estimatorType);

    std::string contactDetectorType;
    if (!YarpUtilities::getElementFromSearchable(config, "contact_detector_type", contactDetectorType))
    {
        yError() << m_printPrefix << "[open] Missing required parameter \"contact_detector_type\"";
        return false;
    }

    if (m_supportedContactDetectorLookup.find(contactDetectorType) == m_supportedContactDetectorLookup.end())
    {
        yError() << m_printPrefix << "[open] Specified \"contact_detector_type\" parameter not in the list of supported estimators."
                 << " Currently supported Contact Detectors: SchmittTrigger.";
        return false;
    }
    m_contactDetectorType = m_supportedContactDetectorLookup.at(contactDetectorType);

    double devicePeriod{0.01};
    if (YarpUtilities::getElementFromSearchable(config, "sampling_period_in_s", devicePeriod))
    {
        setPeriod(devicePeriod);
    }

    if (!setupRobotModel(config))
    {
        return false;
    }

    if (!setupRobotSensorBridge(config))
    {
        return false;
    }

    if (!setupBaseEstimator(config))
    {
        return false;
    }

    if (!YarpUtilities::getElementFromSearchable(config, "publish_to_tf_server", m_publishToTFServer))
     {
         m_publishToTFServer = false;
     }

     if (m_publishToTFServer)
     {
         if (!loadTransformBroadcaster())
         {
             return false;
         }
     }

    return true;
}

bool FloatingBaseEstimatorDevice::setupRobotModel(yarp::os::Searchable& config)
{
    std::string modelFileName;
    std::vector<std::string> jointsList;
    if (!YarpUtilities::getElementFromSearchable(config, "model_file", modelFileName))
    {
        yError() << m_printPrefix << "[setupRobotModel] Missing required parameter \"model_file\"";
        return false;
    }

    if (!YarpUtilities::getVectorFromSearchable(config, "joint_list", jointsList))
    {
        yError() << m_printPrefix << "[setupRobotModel] Missing required parameter \"joint_list\"";
        return false;
    }

    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string modelFilePath{rf.findFileByName(modelFileName)};
    yInfo() << m_printPrefix << "[setupRobotModel] Loading model from " << modelFilePath;

    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadReducedModelFromFile(modelFilePath, jointsList))
    {
        yError() << m_printPrefix << "[setupRobotModel] Could not load robot model";
        return false;
    }

    auto model = modelLoader.model();
    m_kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    m_kinDyn->loadRobotModel(model);

    if (m_kinDyn->isValid())
    {
        m_input.encoders.resize(m_kinDyn->model().getNrOfDOFs());
        m_input.encoders.setZero();
        m_input.encodersSpeed = m_input.encoders;
    }

    YarpUtilities::getElementFromSearchable(config, "left_foot_contact_frame", m_input.lfContactFrameName);
    YarpUtilities::getElementFromSearchable(config, "right_foot_contact_frame", m_input.rfContactFrameName);
    if (!m_kinDyn->model().isFrameNameUsed(m_input.lfContactFrameName) ||
        !m_kinDyn->model().isFrameNameUsed(m_input.rfContactFrameName))
    {
        yError() << m_printPrefix << "[setupRobotModel] Specified contact frame names: "
                 << m_input.lfContactFrameName  << ", "<< m_input.rfContactFrameName << "do not exist in model.";
        return false;
    }

    return m_kinDyn->isValid();
}

bool FloatingBaseEstimatorDevice::setupRobotSensorBridge(yarp::os::Searchable& config)
{
    auto bridgeConfig = config.findGroup("RobotSensorBridge");
    if (bridgeConfig.isNull())
    {
        yError() << m_printPrefix << "[setupRobotSensorBridge] Missing required group \"RobotSensorBridge\"";
        return false;
    }

    std::shared_ptr<YarpImplementation> bridgeHandler = std::make_shared<YarpImplementation>();
    bridgeHandler->set(bridgeConfig);

    m_robotSensorBridge = std::make_unique<YarpSensorBridge>();
    if (!m_robotSensorBridge->initialize(bridgeHandler))
    {
        yError() << m_printPrefix << "[setupRobotSensorBridge] Could not configure RobotSensorBridge";
        return false;
    }

    return true;
}

bool FloatingBaseEstimatorDevice::setupBaseEstimator(yarp::os::Searchable& config)
{
    if (m_estimatorType == BaseEstimatorType::InvEKF &&
        m_contactDetectorType == ContactDetectorType::SchmittTrigger)
    {
        m_invEKFSchmitt = std::make_unique<InvEKFSchmittWrapper>();
    }

    if (m_estimatorType == BaseEstimatorType::LeggedOdom &&
        m_contactDetectorType == ContactDetectorType::SchmittTrigger)
    {
        m_leggedOdomSchmitt = std::make_unique<LOSchmittWrapper>();
    }

    std::shared_ptr<YarpImplementation> originalHandler = std::make_shared<YarpImplementation>();
    originalHandler->set(config);
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

    if (m_leggedOdomSchmitt)
    {
        if (!initializeKIFWrapper(m_leggedOdomSchmitt.get(), parameterHandler,
            m_kinDyn, "LeggedOdometry + SchmittTrigger"))
        {
            return false;
        }
    }

    if (m_invEKFSchmitt)
    {
        if (!initializeKIFWrapper(m_invEKFSchmitt.get(), parameterHandler,
            m_kinDyn, "InvEKF + SchmittTrigger"))
        {
            return false;
        }
    }

    return true;
}

bool FloatingBaseEstimatorDevice::attachAll(const yarp::dev::PolyDriverList & poly)
{
    if (!m_robotSensorBridge->setDriversList(poly))
    {
        yError() << m_printPrefix << "[attachAll] Failed to attach to devices through RobotSensorBridge.";
        return false;
    }

    if (!openCommunications())
    {
        yError() << m_printPrefix << "[attachAll] Could not open ports for publishing outputs.";
        return false;
    }

    this->start();
    return true;
}

bool FloatingBaseEstimatorDevice::openCommunications()
{
    if (!openBufferedSigPort(m_comms.floatingBaseStatePort, m_portPrefix, "/floating_base/state:o"))
    {
        return false;
    }

    openBufferedSigPort(m_comms.internalStateAndStdDevPort, m_portPrefix, "/internal_state/stateAndStdDev:o");
    openBufferedSigPort(m_comms.contactStatePort, m_portPrefix, "/contacts/stateAndNormalForce:o");
    return true;
}

bool FloatingBaseEstimatorDevice::openBufferedSigPort(yarp::os::BufferedPort<yarp::sig::Vector>& port,
                                                      const std::string& portPrefix,
                                                      const std::string& address)
{
    bool ok{false};
    ok = port.open(portPrefix + address);
    if (!ok)
    {
        yError() << m_printPrefix << "[openBufferedSigPort] error opening port " << portPrefix + address;
        return false;
    }
    return true;
}

void FloatingBaseEstimatorDevice::run()
{
    // update estimator measurements
    if (!updateMeasurements())
    {
        yWarning() << m_printPrefix << "[run] Measurement updates failed.";
        return;
    }

    // advance estimator
    if (!updateEstimator())
    {
        yWarning() << m_printPrefix << "[run] Estimator updates failed.";
        return;
    }

    publish();
}

bool FloatingBaseEstimatorDevice::updateMeasurements()
{
    bool ok{true};

    // advance sensor bridge
    if (!m_robotSensorBridge->advance())
    {
        yWarning() << m_printPrefix << "[updateMeasurements] advance Sensor bridge failed.";
        return false;
    }

    m_input.ts = yarp::os::Time::now();
    ok = ok && updateContactWrenches();
    ok = ok && updateInertialBuffers();
    ok = ok && updateKinematics();

    return ok;
}

bool FloatingBaseEstimatorDevice::updateInertialBuffers()
{
    Eigen::Matrix<double, 12, 1> imuMeasure;
    if (!m_robotSensorBridge->getIMUMeasurement(m_baseLinkImuName, imuMeasure))
    {
        return false;
    }

    const int accOffset{3};
    const int gyroOffset{6};
    m_input.acc = imuMeasure.segment<3>(accOffset);
    m_input.gyro = imuMeasure.segment<3>(gyroOffset);

    return true;
}

bool FloatingBaseEstimatorDevice::updateContactWrenches()
{
    bool ok{true};
    Eigen::Matrix<double, 6, 1> lfWrench, rfWrench;
    lfWrench.setZero();
    rfWrench.setZero();

    if (m_robotModelUsesFrontRearFootFTs)
    {
        // in case of iCubGenova09, sum the front and rear contact wrenches
        // no need for an adjoint wrench transform since, both the
        // wrenches are already expressed in the sole frame
        // see https://github.com/robotology/whole-body-estimators/blob/6841cdcc49a30c2ee59a6d7e5437d9b24b5b2d4f/devices/wholeBodyDynamics/app/wholebodydynamics-icub3-external-sim.xml#L31-L34
        for (auto& wrenchName : m_leftFootWrenchNames)
        {
            Eigen::Matrix<double, 6, 1> tempWrench;
            ok = ok && m_robotSensorBridge->getCartesianWrench(wrenchName, tempWrench);
            lfWrench += tempWrench;
        }

        for (auto& wrenchName : m_rightFootWrenchNames)
        {
            Eigen::Matrix<double, 6, 1> tempWrench;
            ok = ok && m_robotSensorBridge->getCartesianWrench(wrenchName, tempWrench);
            rfWrench += tempWrench;
        }
    }
    else
    {
        ok = ok && m_robotSensorBridge->getCartesianWrench(m_leftFootWrenchNames[0], lfWrench);
        ok = ok && m_robotSensorBridge->getCartesianWrench(m_rightFootWrenchNames[0], rfWrench);
    }

    if (!ok)
    {
        return false;
    }

    m_input.contactWrenches.clear();
    m_input.contactWrenches[m_input.lfContactFrameName] = lfWrench;
    m_input.contactWrenches[m_input.rfContactFrameName] = rfWrench;

    return true;
}

bool FloatingBaseEstimatorDevice::updateKinematics()
{
    if (!m_robotSensorBridge->getJointPositions(m_input.encoders))
    {
        return false;
    }

    if (!m_robotSensorBridge->getJointVelocities(m_input.encodersSpeed))
    {
        return false;
    }

    return true;
}

bool FloatingBaseEstimatorDevice::updateEstimator()
{
    bool ok{true};
    if (m_leggedOdomSchmitt)
    {
        ok = updateKIFWrapper(m_leggedOdomSchmitt.get(), m_input, m_output);
    }

    if (m_invEKFSchmitt)
    {
        ok = updateKIFWrapper(m_invEKFSchmitt.get(), m_input, m_output);
    }

    return ok;
}

void FloatingBaseEstimatorDevice::publish()
{
    publishBaseLinkState(m_output.estimatorOut);
    publishInternalStateAndStdDev(m_output.estimatorOut);

    m_currentlFootState = m_output.contactDetectorOut.at(m_input.lfContactFrameName).isActive;
    m_currentrFootState = m_output.contactDetectorOut.at(m_input.rfContactFrameName).isActive;
    m_currentlContactNormal = m_input.contactWrenches.at(m_input.lfContactFrameName)(2);
    m_currentrContactNormal = m_input.contactWrenches.at(m_input.rfContactFrameName)(2);
    publishFootContactStatesAndNormalForces();
}

void FloatingBaseEstimatorDevice::publishBaseLinkState(const FloatingBaseEstimators::Output& estimatorOut)
{
    size_t stateVecSize{12};
    size_t rpyOffset{3};
    size_t linVelOffset{6};
    size_t angVelOffset{9};

    iDynTree::Rotation baseRdyn;
    iDynTree::toEigen(baseRdyn) = estimatorOut.basePose.quat().toRotationMatrix();
    m_basePos = estimatorOut.basePose.translation();
    m_baseRPY = iDynTree::toEigen(baseRdyn.asRPY()); // rpy euler angles in xyz convention
    m_baseLinearVel = estimatorOut.baseTwist.head<3>();
    m_baseAngularVel = estimatorOut.baseTwist.tail<3>();

    yarp::sig::Vector& stateVec = m_comms.floatingBaseStatePort.prepare();
    stateVec.clear();
    stateVec.resize(stateVecSize);

    for (size_t idx = 0; idx < rpyOffset; idx++) { stateVec(idx) = m_basePos(idx); }
    for (size_t idx = rpyOffset; idx < linVelOffset; idx++) { stateVec(idx) = m_baseRPY(idx - rpyOffset); }
    for (size_t idx = linVelOffset; idx < angVelOffset; idx++) { stateVec(idx) = m_baseLinearVel(idx - linVelOffset); }
    for (size_t idx = angVelOffset; idx < stateVecSize; idx++) { stateVec(idx) = m_baseAngularVel(idx - angVelOffset); }

    m_comms.floatingBaseStatePort.write();

    yarp::sig::Matrix basePoseYARP;
    size_t dim{4};
    basePoseYARP.resize(dim, dim);
    memcpy(basePoseYARP.data(),estimatorOut.basePose.transform().data(),dim*dim*sizeof(double));

    if (m_publishToTFServer && m_transformInterface != nullptr)
    {
        if (!m_transformInterface->setTransform("/world", "/base_link", basePoseYARP))
        {
            yError() << "[FloatingBaseEstimatorDevice] Could not publish measured base pose transform from  primary IMU";
        }
    }
}

void FloatingBaseEstimatorDevice::publishInternalStateAndStdDev(const FloatingBaseEstimators::Output& estimatorOut)
{
    size_t vecSize{27};

    const auto& s = estimatorOut.state;
    const auto& d = estimatorOut.stateStdDev;

    iDynTree::Rotation Rdyn;
    iDynTree::toEigen(Rdyn) = s.imuOrientation.toRotationMatrix();
    Eigen::Vector3d imuRPY = iDynTree::toEigen(Rdyn.asRPY());

    iDynTree::toEigen(Rdyn) = s.rContactFrameOrientation.toRotationMatrix();
    Eigen::Vector3d rfRPY = iDynTree::toEigen(Rdyn.asRPY());

    iDynTree::toEigen(Rdyn) = s.lContactFrameOrientation.toRotationMatrix();
    Eigen::Vector3d lfRPY = iDynTree::toEigen(Rdyn.asRPY());

    Eigen::VectorXd internalstateAndStdDev;
    internalstateAndStdDev.resize(vecSize + vecSize);
    internalstateAndStdDev << s.imuPosition, imuRPY, s.imuLinearVelocity,
                              s.rContactFramePosition, rfRPY,
                              s.lContactFramePosition, lfRPY,
                              s.accelerometerBias, s.gyroscopeBias,
                              d.imuPosition, d.imuOrientation, d.imuLinearVelocity,
                              d.rContactFramePosition, d.rContactFrameOrientation,
                              d.lContactFramePosition, d.lContactFrameOrientation,
                              d.accelerometerBias, d.gyroscopeBias;

    yarp::sig::Vector& stateAndStdDev = m_comms.internalStateAndStdDevPort.prepare();
    stateAndStdDev.clear();
    stateAndStdDev.resize(vecSize + vecSize);

    for (size_t idx = 0; idx < 2*vecSize; idx++)
    {
        stateAndStdDev(idx) = internalstateAndStdDev[idx];
    }

    m_comms.internalStateAndStdDevPort.write();
}

void FloatingBaseEstimatorDevice::publishFootContactStatesAndNormalForces()
{
    yarp::sig::Vector& contactVec = m_comms.contactStatePort.prepare();
    contactVec.clear();

    contactVec.resize(4);
    const int scaling_const_for_visualization{300};
    contactVec(0) = scaling_const_for_visualization*static_cast<int>(m_currentlFootState);
    contactVec(1) = scaling_const_for_visualization*static_cast<int>(m_currentrFootState);
    contactVec(2) = m_currentlContactNormal;
    contactVec(3) = m_currentrContactNormal;
    m_comms.contactStatePort.write();
}

bool FloatingBaseEstimatorDevice::detachAll()
{
    std::lock_guard<std::mutex> guard(m_deviceMutex);
    if (this->isRunning())
    {
        this->stop();
    }

    yInfo() << m_printPrefix << " Device stopped.";
    return true;
}

void FloatingBaseEstimatorDevice::closeBufferedSigPort(yarp::os::BufferedPort<yarp::sig::Vector>& port)
{
    if (!port.isClosed())
    {
        port.close();
    }

    if (m_publishToTFServer)
    {
        m_transformInterface = nullptr;
    }
}

void FloatingBaseEstimatorDevice::closeCommunications()
{
    closeBufferedSigPort(m_comms.floatingBaseStatePort);
    closeBufferedSigPort(m_comms.internalStateAndStdDevPort);
    closeBufferedSigPort(m_comms.contactStatePort);
}

bool FloatingBaseEstimatorDevice::close()
{
    std::lock_guard<std::mutex> guard(m_deviceMutex);
    yInfo() << m_printPrefix << " Closing BipedalLocomotion::FloatingBaseEstimatorDevice.";
    closeCommunications();
    return true;
}

bool FloatingBaseEstimatorDevice::loadTransformBroadcaster()
{
    yarp::os::Property tfBroadcasterSettings{{"device", yarp::os::Value("transformClient")},
                                             {"remote", yarp::os::Value("/transformServer")},
                                             {"local", yarp::os::Value(m_portPrefix + "/transformClient")}};

    if (!m_transformBroadcaster.open(tfBroadcasterSettings))
    {
        yError() << "[FloatingBaseEstimatorDevice][loadTransformBroadcaster] could not open transform broadcaster.";
        return false;
    }

    if (!m_transformBroadcaster.view(m_transformInterface))
    {
        yError() << "[FloatingBaseEstimatorDevice][loadTransformBroadcaster] could not access transform interface";
        return false;
    }

    return true;
}
