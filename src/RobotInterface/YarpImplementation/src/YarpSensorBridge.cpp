/**
 * @file YarpSensorBridge.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/RobotInterface/YarpSensorBridgeImpl.h>
#include <yarp/eigen/Eigen.h>

using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::GenericContainer;
using namespace BipedalLocomotion::ParametersHandler;

YarpSensorBridge::YarpSensorBridge() : m_pimpl(std::make_unique<Impl>())
{
}

YarpSensorBridge::~YarpSensorBridge() = default;

bool YarpSensorBridge::initialize(std::weak_ptr<IParametersHandler> handler)
{
    constexpr std::string_view logPrefix = "[YarpSensorBridge::initialize] ";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        std::cerr << logPrefix << "The handler is not pointing to an already initialized memory."
                  << std ::endl;
        return false;
    }


    if(!ptr->getParameter("check_for_nan", m_pimpl->checkForNAN))
    {
        std::cerr << logPrefix << "Unable to get check_for_nan."
                  << std ::endl;
        return false;
    }

    bool ret{true};
    ret = m_pimpl->subConfigLoader("stream_joint_states", "RemoteControlBoardRemapper",
                                    &YarpSensorBridge::Impl::configureRemoteControlBoardRemapper,
                                    handler,
                                    m_pimpl->metaData,
                                    m_pimpl->metaData.bridgeOptions.isKinematicsEnabled);
    if (!ret)
    {
        std::cout << logPrefix << " Skipping the configuration of RemoteControlBoardRemapper. YarpSensorBridge will not stream relevant measures." << std::endl;
    }

    bool useInertialSensors{false};
    ret = m_pimpl->subConfigLoader("stream_inertials", "InertialSensors",
                                    &YarpSensorBridge::Impl::configureInertialSensors,
                                    handler,
                                    m_pimpl->metaData,
                                    useInertialSensors);
    if (!ret)
    {
        std::cout << logPrefix << " Skipping the configuration of InertialSensors. YarpSensorBridge will not stream relevant measures." << std::endl;
    }

    ret = m_pimpl->subConfigLoader("stream_forcetorque_sensors", "SixAxisForceTorqueSensors",
                                    &YarpSensorBridge::Impl::configureSixAxisForceTorqueSensors,
                                    handler,
                                    m_pimpl->metaData,
                                    m_pimpl->metaData.bridgeOptions.isSixAxisForceTorqueSensorEnabled);
    if (!ret)
    {
        std::cout << logPrefix << " Skipping the configuration of SixAxisForceTorqueSensors. YarpSensorBridge will not stream relevant measures." << std::endl;
    }

    ret = m_pimpl->subConfigLoader("stream_cartesian_wrenches", "CartesianWrenches",
                                    &YarpSensorBridge::Impl::configureCartesianWrenches,
                                    handler,
                                    m_pimpl->metaData,
                                    m_pimpl->metaData.bridgeOptions.isCartesianWrenchEnabled);
    if (!ret)
    {
        std::cout << logPrefix << " Skipping the configuration of CartesianWrenches. YarpSensorBridge will not stream relevant measures." << std::endl;
    }

    m_pimpl->bridgeInitialized = true;
    return true;
}

bool YarpSensorBridge::setDriversList(const yarp::dev::PolyDriverList& deviceDriversList)
{    
    constexpr std::string_view logPrefix = "[YarpSensorBridge::setDriversList] ";

    if (!m_pimpl->bridgeInitialized)
    {
        std::cerr << logPrefix << "Please initialize YarpSensorBridge before calling setDriversList(...)."
                  << std ::endl;
        return false;
    }
        
    bool ret{true};
    ret = ret && m_pimpl->attachRemappedRemoteControlBoard(deviceDriversList);
    ret = ret && m_pimpl->attachAllInertials(deviceDriversList);
    ret = ret && m_pimpl->attachAllSixAxisForceTorqueSensors(deviceDriversList);
    ret = ret && m_pimpl->attachCartesianWrenchInterface(deviceDriversList);

    if (!ret)
    {
        std::cerr << logPrefix << "Failed to attach to one or more device drivers."
                  << std ::endl;
        return false;
    }
    m_pimpl->driversAttached = true;
    return true;
}

bool YarpSensorBridge::advance()
{
    constexpr std::string_view logPrefix = "[YarpSensorBridge::advance] ";
    if (!m_pimpl->checkValid("[YarpSensorBridge::advance]"))
    {
        std::cerr << logPrefix << "Please initialize and set drivers list before running advance()."
                  << std ::endl;
        return false;
    }

    m_pimpl->readAllSensors(m_pimpl->failedSensorReads);

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

bool YarpSensorBridge::getJointPosition(const std::string& jointName,
                                        double& jointPosition,
                                        double* receiveTimeInSeconds)
{
    int idx;
    if (!m_pimpl->getIndexFromVector(m_pimpl->metaData.sensorsList.jointsList,
                                     jointName, idx))
    {
        std::cerr << "[YarpSensorBridge::getJointVelocity] " << jointName
                  <<  " could not be found in the configured list of joints" << std::endl;
        return false;
    }

    jointPosition = m_pimpl->controlBoardRemapperMeasures.jointPositions[idx];
    receiveTimeInSeconds = &m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getJointPositions(Eigen::Ref<Eigen::VectorXd> jointPositions,
                                         double* receiveTimeInSeconds)
{

    jointPositions = yarp::eigen::toEigen(m_pimpl->controlBoardRemapperMeasures.jointPositions);
    receiveTimeInSeconds = &m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;
    return true;
}

bool YarpSensorBridge::getJointVelocity(const std::string& jointName,
                                        double& jointVelocity,
                                        double* receiveTimeInSeconds)
{
    int idx;
    if (!m_pimpl->getIndexFromVector(m_pimpl->metaData.sensorsList.jointsList,
                                     jointName, idx))
    {
        std::cerr << "[YarpSensorBridge::getJointVelocity] " << jointName
                  <<  " could not be found in the configured list of joints" << std::endl;
        return false;
    }

    jointVelocity = m_pimpl->controlBoardRemapperMeasures.jointVelocities[idx];
    receiveTimeInSeconds = &m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getJointVelocities(Eigen::Ref<Eigen::VectorXd> jointVelocties,
                                          double* receiveTimeInSeconds)
{
    jointVelocties = yarp::eigen::toEigen(m_pimpl->controlBoardRemapperMeasures.jointVelocities);
    receiveTimeInSeconds = &m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;
    return true;
}

bool YarpSensorBridge::getIMUMeasurement(const std::string& imuName,
                                         Eigen::Ref<Vector12d> imuMeasurement,
                                         double* receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpSensorBridge::getIMUMeasurement ",
                                       m_pimpl->wholeBodyIMUMeasures, imuName))
    {
        return false;
    }

    auto iter = m_pimpl->wholeBodyIMUMeasures.find(imuName);
    imuMeasurement = yarp::eigen::toEigen(iter->second.first);
    receiveTimeInSeconds = &iter->second.second;
    return true;
}

bool YarpSensorBridge::getLinearAccelerometerMeasurement(const std::string& accName,
                                                         Eigen::Ref<Eigen::Vector3d> accMeasurement,
                                                         double* receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpSensorBridge::getLinearAccelerometerMeasurement ",
                                           m_pimpl->wholeBodyInertialMeasures, accName))
    {
        return false;
    }

    auto iter = m_pimpl->wholeBodyInertialMeasures.find(accName);
    accMeasurement = yarp::eigen::toEigen(iter->second.first);
    receiveTimeInSeconds = &iter->second.second;
    return true;
}

bool YarpSensorBridge::getGyroscopeMeasure(const std::string& gyroName,
                                           Eigen::Ref<Eigen::Vector3d> gyroMeasurement,
                                           double* receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpSensorBridge::getGyroscopeMeasure ",
                                           m_pimpl->wholeBodyInertialMeasures, gyroName))
    {
        return false;
    }

    auto iter = m_pimpl->wholeBodyInertialMeasures.find(gyroName);
    gyroMeasurement = yarp::eigen::toEigen(iter->second.first);
    receiveTimeInSeconds = &iter->second.second;
    return true;
}


bool YarpSensorBridge::getOrientationSensorMeasurement(const std::string& rpyName,
                                                       Eigen::Ref<Eigen::Vector3d> rpyMeasurement,
                                                       double* receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpSensorBridge::getOrientationSensorMeasurement ",
                                           m_pimpl->wholeBodyInertialMeasures, rpyName))
    {
        return false;
    }

    auto iter = m_pimpl->wholeBodyInertialMeasures.find(rpyName);
    rpyMeasurement = yarp::eigen::toEigen(iter->second.first);
    receiveTimeInSeconds = &iter->second.second;
    return true;
}

bool YarpSensorBridge::getMagnetometerMeasurement(const std::string& magName,
                                                  Eigen::Ref<Eigen::Vector3d> magMeasurement,
                                                  double* receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpSensorBridge::getMagnetometerMeasurement ",
                                           m_pimpl->wholeBodyInertialMeasures, magName))
    {
        return false;
    }

    auto iter = m_pimpl->wholeBodyInertialMeasures.find(magName);
    magMeasurement = yarp::eigen::toEigen(iter->second.first);
    receiveTimeInSeconds = &iter->second.second;
    return true;
}

bool YarpSensorBridge::getSixAxisForceTorqueMeasurement(const std::string& ftName,
                                                        Eigen::Ref<Vector6d> ftMeasurement,
                                                        double* receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpSensorBridge::getSixAxisForceTorqueMeasurement ",
                                           m_pimpl->wholeBodyFTMeasures, ftName))
    {
        return false;
    }

    auto iter = m_pimpl->wholeBodyFTMeasures.find(ftName);
    ftMeasurement = yarp::eigen::toEigen(iter->second.first);
    receiveTimeInSeconds = &iter->second.second;
    return true;
}

bool YarpSensorBridge::getCartesianWrench(const std::string& cartesianWrenchName,
                                          Eigen::Ref<Vector6d> cartesianWrenchMeasurement,
                                          double* receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpSensorBridge::getCartesianWrench ",
                                           m_pimpl->wholeBodyCartesianWrenchMeasures, cartesianWrenchName))
    {
        return false;
    }

    auto iter = m_pimpl->wholeBodyCartesianWrenchMeasures.find(cartesianWrenchName);
    cartesianWrenchMeasurement = yarp::eigen::toEigen(iter->second.first);
    receiveTimeInSeconds = &iter->second.second;
    return true;
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
