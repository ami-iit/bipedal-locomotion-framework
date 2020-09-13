/**
 * @file YarpSensorBridge.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/RobotInterface/YarpSensorBridgeImpl.h>


YarpSensorBridge::YarpSensorBridge() : m_pimpl(std::make_unique<Impl>())
{
};

YarpSensorBridge::~YarpSensorBridge() = default;

bool YarpSensorBridge::initialize(std::weak_ptr<IParametersHandler> handler)
{
    constexpr std::string_view errorPrefix = "[YarpSensorBridge::initialize] ";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        std::cerr << errorPrefix << "The handler is not pointing to an already initialized memory."
                  << std ::endl;
        return false;
    }

    int temp;
    ptr->getParameter("sensor_dry_run", temp);
    m_pimpl->sensorDryRunEnabled = static_cast<bool>(temp);

    bool ret{true};
    ret = ret && m_pimpl->subConfigLoader("stream_joint_states", "RemoteControlBoardRemapper",
                                          &YarpSensorBridge::Impl::configureRemoteControlBoardRemapper,
                                          handler,
                                          m_pimpl->metaData,
                                          m_pimpl->metaData.bridgeOptions.isKinematicsEnabled);

    bool useInertialSensors{false};
    ret = ret && m_pimpl->subConfigLoader("stream_inertials", "InertialSensors",
                                          &YarpSensorBridge::Impl::configureInertialSensors,
                                          handler,
                                          m_pimpl->metaData,
                                          useInertialSensors);

    ret = ret && m_pimpl->subConfigLoader("stream_forcetorque_sensors", "SixAxisForceTorqueSensors",
                                          &YarpSensorBridge::Impl::configureSixAxisForceTorqueSensors,
                                          handler,
                                          m_pimpl->metaData,
                                          m_pimpl->metaData.bridgeOptions.isSixAxisForceTorqueSensorEnabled);

    ret = ret && m_pimpl->subConfigLoader("stream_cartesian_wrenches", "CartesianWrenches",
                                          &YarpSensorBridge::Impl::configureCartesianWrenches,
                                          handler,
                                          m_pimpl->metaData,
                                          m_pimpl->metaData.bridgeOptions.isCartesianWrenchEnabled);

    bool useCameras{false};
    ret = ret && m_pimpl->subConfigLoader("stream_cameras", "Cameras",
                                          &YarpSensorBridge::Impl::configureCameras,
                                          handler,
                                          m_pimpl->metaData,
                                          useCameras);


    m_pimpl->bridgeInitialized = true;
    return true;
}

bool YarpSensorBridge::setDriversList(const yarp::dev::PolyDriverList& deviceDriversList)
{
    m_pimpl->driversAttached = true;
    return true;
}

bool YarpSensorBridge::advance()
{
    return true;
}

bool YarpSensorBridge::isValid() const
{
    return m_pimpl->checkValid("[YarpSensorBridge::isValid]");
}

const SensorBridgeMetaData& YarpSensorBridge::get() const { return m_pimpl->metaData; }

bool YarpSensorBridge::getJointsList(std::vector<std::string>& jointsList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getJointsList]"))
    {
        return false;
    }
    jointsList = m_pimpl->metaData.sensorsList.jointsList;
    return true;
}

bool YarpSensorBridge::getIMUsList(std::vector<std::string>& IMUsList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getIMUsList]"))
    {
        return false;
    }
    IMUsList = m_pimpl->metaData.sensorsList.IMUsList;
    return true;
}

bool YarpSensorBridge::getLinearAccelerometersList(std::vector<std::string>& linearAccelerometersList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getLinearAccelerometersList]"))
    {
        return false;
    }
    linearAccelerometersList = m_pimpl->metaData.sensorsList.linearAccelerometersList;
    return true;
}

bool YarpSensorBridge::getGyroscopesList(std::vector<std::string>& gyroscopesList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getGyroscopesList]"))
    {
        return false;
    }
    gyroscopesList = m_pimpl->metaData.sensorsList.gyroscopesList;
    return true;
}

bool YarpSensorBridge::getOrientationSensorsList(std::vector<std::string>& orientationSensorsList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getOrientationSensorsList]"))
    {
        return false;
    }
    orientationSensorsList = m_pimpl->metaData.sensorsList.orientationSensorsList;
    return true;
}

bool YarpSensorBridge::getMagnetometersList(std::vector<std::string>& magnetometersList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getMagnetometersList]"))
    {
        return false;
    }
    magnetometersList = m_pimpl->metaData.sensorsList.magnetometersList;
    return true;
}

bool YarpSensorBridge::getSixAxisForceTorqueSensorsList(std::vector<std::string>& sixAxisForceTorqueSensorsList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getSixAxisForceTorqueSensorsList]"))
    {
        return false;
    }
    sixAxisForceTorqueSensorsList = m_pimpl->metaData.sensorsList.sixAxisForceTorqueSensorsList;
    return true;
}

bool YarpSensorBridge::getCartesianWrenchesList(std::vector<std::string>& cartesianWrenchesList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getCartesianWrenchesList]"))
    {
        return false;
    }
    cartesianWrenchesList = m_pimpl->metaData.sensorsList.cartesianWrenchesList;
    return true;
}

bool YarpSensorBridge::getRGBCamerasList(std::vector<std::string>& rgbCamerasList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getRGBCamerasList]"))
    {
        return false;
    }
    rgbCamerasList = m_pimpl->metaData.sensorsList.rgbCamerasList;
    return true;
}

bool YarpSensorBridge::getDepthCamerasList(std::vector<std::string>& depthCamerasList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getDepthCamerasList]"))
    {
        return false;
    }
    depthCamerasList = m_pimpl->metaData.sensorsList.depthCamerasList;
    return true;
}

bool YarpSensorBridge::getJointPosition(const std::string& jointName,
                                        double& jointPosition,
                                        double* receiveTimeInSeconds)
{
    return true;
}

bool YarpSensorBridge::getJointPositions(Eigen::Ref<Eigen::VectorXd> jointPositions,
                                         double* receiveTimeInSeconds)
{
    return true;
}

bool YarpSensorBridge::getJointVelocity(const std::string& jointName,
                                        double& jointVelocity,
                                        double* receiveTimeInSeconds)
{
    return true;
}


bool YarpSensorBridge::getJointVelocities(Eigen::Ref<Eigen::VectorXd> jointVelocties,
                                          double* receiveTimeInSeconds)
{
    return true;
}

bool YarpSensorBridge::getIMUMeasurement(const std::string& imuName,
                                         Eigen::Ref<Vector12d> imuMeasurement,
                                         double* receiveTimeInSeconds)
{
    return true;
}

bool YarpSensorBridge::getLinearAccelerometerMeasurement(const std::string& accName,
                                                         Eigen::Ref<Eigen::Vector3d> accMeasurement,
                                                         double* receiveTimeInSeconds)
{
    return true;
}

bool YarpSensorBridge::getGyroscopeMeasure(const std::string& gyroName,
                                           Eigen::Ref<Eigen::Vector3d> gyroMeasurement,
                                           double* receiveTimeInSeconds)
{
    return true;
}


bool YarpSensorBridge::getOrientationSensorMeasurement(const std::string& rpyName,
                                                       Eigen::Ref<Eigen::Vector3d> rpyMeasurement,
                                                       double* receiveTimeInSeconds)
{
    return true;
}

bool YarpSensorBridge::getMagnetometerMeasurement(const std::string& magName,
                                                  Eigen::Ref<Eigen::Vector3d> magMeasurement,
                                                  double* receiveTimeInSeconds)
{
    return true;
}

bool YarpSensorBridge::getSixAxisForceTorqueMeasurement(const std::string& ftName,
                                                        Eigen::Ref<Vector6d> ftMeasurement,
                                                        double* receiveTimeInSeconds)
{
    return true;
}

bool YarpSensorBridge::getCartesianWrench(const std::string& cartesianWrenchName,
                                          Eigen::Ref<Vector6d> cartesianWrenchMeasurement,
                                          double* receiveTimeInSeconds)
{
    return true;
}

bool YarpSensorBridge::getColorImage(const std::string& camName,
                                     Eigen::Ref<Eigen::MatrixXd> colorImg,
                                     double* receiveTimeInSeconds)
{
    return true;
}

bool YarpSensorBridge::getDepthImage(const std::string& camName,
                                     Eigen::Ref<Eigen::MatrixXd> depthImg,
                                     double* receiveTimeInSeconds)
{
    return true;
}

bool YarpSensorBridge::populateSensorBridgeOptionsFromConfig(std::weak_ptr<IParametersHandler> handler,
                                                             SensorBridgeOptions& sensorBridgeOptions)
{
    constexpr std::string_view error = "[YarpSensorBridge::populateSensorBridgeOptionsFromConfig] Unused method.";
    std::cerr << error <<  std::endl;
    return false;
}

bool YarpSensorBridge::populateSensorListsFromConfig(std::weak_ptr<IParametersHandler> handler,
                                                     const SensorBridgeOptions& sensorBridgeOptions,
                                                     SensorLists& sensorLists)
{
    constexpr std::string_view error = "[YarpSensorBridge::populateSensorListsFromConfig] Unused method.";
    std::cerr << error <<  std::endl;
    return false;
}

bool YarpSensorBridge::getThreeAxisForceTorqueMeasurement(const std::string& ftName,
                                                          Eigen::Ref<Eigen::Vector3d> ftMeasurement,
                                                          double* receiveTimeInSeconds)
{
    constexpr std::string_view error = "[YarpSensorBridge::getThreeAxisForceTorqueMeasurement] Currently unimplemented";
    std::cerr << error <<  std::endl;
    return false;
}

bool YarpSensorBridge::getThreeAxisForceTorqueSensorsList(std::vector<std::string>& threeAxisForceTorqueSensorsList)
{
    constexpr std::string_view error = "[YarpSensorBridge::getThreeAxisForceTorqueSensorsList] Currently unimplemented";
    std::cerr << error <<  std::endl;

    return false;
}
