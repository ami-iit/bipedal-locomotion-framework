/**
 * @copyright 2020, 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iomanip>
#include <tuple>

#include <BipedalLocomotion/Conversions/matioCppConversions.h>
#include <BipedalLocomotion/FTIMULoggerDevice.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/YarpUtilities/Helper.h>

#include <yarp/os/LogStream.h>

using namespace BipedalLocomotion::YarpUtilities;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::Conversions;
using namespace BipedalLocomotion;

FTIMULoggerDevice::FTIMULoggerDevice(double period, yarp::os::ShouldUseSystemClock useSystemClock)
    : yarp::os::PeriodicThread(period, useSystemClock)
{
}

FTIMULoggerDevice::FTIMULoggerDevice()
    : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No)
{
}

FTIMULoggerDevice::~FTIMULoggerDevice() = default;

bool FTIMULoggerDevice::open(yarp::os::Searchable& config)
{
    double devicePeriod{0.01};
    if (YarpUtilities::getElementFromSearchable(config, "sampling_period_in_s", devicePeriod))
    {
        this->setPeriod(devicePeriod);
    }

    if (!setupRobotSensorBridge(config))
    {
        return false;
    }

    return true;
}

bool FTIMULoggerDevice::setupRobotSensorBridge(yarp::os::Searchable& config)
{
    auto bridgeConfig = config.findGroup("RobotSensorBridge");
    if (bridgeConfig.isNull())
    {
        yError() << "[FTIMULoggerDevice][setupRobotSensorBridge] Missing required group "
                    "\"RobotSensorBridge\"";
        return false;
    }

    auto originalHandler = std::make_shared<YarpImplementation>();
    originalHandler->set(bridgeConfig);

    m_robotSensorBridge = std::make_unique<YarpSensorBridge>();
    if (!m_robotSensorBridge->initialize(originalHandler))
    {
        yError() << "[FTIMULoggerDevice][setupRobotSensorBridge] Could not configure "
                    "RobotSensorBridge";
        return false;
    }

    return true;
}

bool FTIMULoggerDevice::attachAll(const yarp::dev::PolyDriverList& poly)
{
    if (!m_robotSensorBridge->setDriversList(poly))
    {
        yError() << "[FTIMULoggerDevice][attachAll] Could not attach drivers list to sensor "
                    "bridge.";
        return false;
    }

    if (!m_robotSensorBridge->getJointsList(m_jointNames))
    {
        yError() << "[FTIMULoggerDevice][attachAll] Could not get the joints list.";
        return false;
    }

    if (!m_robotSensorBridge->getSixAxisForceTorqueSensorsList(m_FTNames))
    {
        yError() << "[FTIMULoggerDevice][attachAll] Could not get the six axis force torque "
                    "sensors list.";
        return false;
    }

    if (!m_robotSensorBridge->getGyroscopesList(m_gyroNames))
    {
        yError() << "[FTIMULoggerDevice][attachAll] Could not get the gyroscope list.";
        return false;
    }

    if (!m_robotSensorBridge->getLinearAccelerometersList(m_accelerometerNames))
    {
        yError() << "[FTIMULoggerDevice][attachAll] Could not get the accelerometers list.";
        return false;
    }

    if (!m_robotSensorBridge->getOrientationSensorsList(m_orientationNames))
    {
        yError() << "[FTIMULoggerDevice][attachAll] Could not get the orientation sensor list.";
        return false;
    }

    if (!m_robotSensorBridge->getIMUsList(m_IMUNames))
    {
        yError() << "[FTIMULoggerDevice][attachAll] Could not get the IMUs list.";
        return false;
    }

    // prepare the unordered_maps
    for (const auto& ft : m_FTNames)
    {
        m_fts[ft] = Eigen::MatrixXd();
    }

    for (const auto& gyro : m_gyroNames)
    {
        m_gyros[gyro] = Eigen::MatrixXd();
    }

    for (const auto& acc : m_accelerometerNames)
    {
        m_accelerometers[acc] = Eigen::MatrixXd();
    }

    for (const auto& orientation : m_orientationNames)
    {
        m_orientations[orientation] = Eigen::MatrixXd();
    }

    // an IMU consist in gyro acc and orient
    for (const auto& IMU : m_IMUNames)
    {
        m_gyros[IMU] = Eigen::MatrixXd();
        m_accelerometers[IMU] = Eigen::MatrixXd();
        m_orientations[IMU] = Eigen::MatrixXd();
    }

    m_dofs = m_jointNames.size();

    m_jointState["joint_positions"] = Eigen::MatrixXd();
    m_jointState["joint_velocities"] = Eigen::MatrixXd();

    start();
    return true;
}

void FTIMULoggerDevice::unpackIMU(Eigen::Ref<const analog_sensor_t> signal,
                                  Eigen::Ref<accelerometer_t> accelerometer,
                                  Eigen::Ref<gyro_t> gyro,
                                  Eigen::Ref<orientation_t> orientation)
{
    // theThe output consists in 12 double, organized as follows:
    //  euler angles [3]
    // linear acceleration [3]
    // angular speed [3]
    // magnetic field [3]
    // http://wiki.icub.org/wiki/Inertial_Sensor
    orientation = signal.segment<3>(0);
    accelerometer = signal.segment<3>(3);
    gyro = signal.segment<3>(6);
}

void FTIMULoggerDevice::run()
{
    // get the data
    if (!m_robotSensorBridge->advance())
    {
        yError() << "[FTIMULoggerDevice][run] could not advance sensor bridge.";
    }

    const auto bufferSize = m_time.rows();

    // get the FT
    for (const auto& key : m_FTNames)
    {
        m_fts.at(key).conservativeResize(6, bufferSize + 1);
        if (!m_robotSensorBridge->getSixAxisForceTorqueMeasurement(key,
                                                                   m_fts.at(key).col(bufferSize),
                                                                   m_timeNow))
        {
            yError() << "[FTIMULoggerDevice][run] Unable to get the ft named: " << key;
        }
    }

    for (const auto& key : m_orientationNames)
    {
        m_orientations.at(key).conservativeResize(3, bufferSize + 1);
        if (!m_robotSensorBridge->getOrientationSensorMeasurement(key,
                                                                  m_orientations.at(key).col(
                                                                      bufferSize),
                                                                  m_timeNow))
        {
            yError() << "[FTIMULoggerDevice][run] Unable to get the orientation named: " << key;
        }
    }

    for (const auto& key : m_accelerometerNames)
    {
        m_accelerometers.at(key).conservativeResize(3, bufferSize + 1);
        if (!m_robotSensorBridge->getLinearAccelerometerMeasurement(key,
                                                                    m_accelerometers.at(key).col(
                                                                        bufferSize),
                                                                    m_timeNow))
        {
            yError() << "[FTIMULoggerDevice][run] Unable to get the accelerometer named: " << key;
        }
    }

    for (const auto& key : m_gyroNames)
    {
        m_gyros.at(key).conservativeResize(3, bufferSize + 1);
        if (!m_robotSensorBridge->getGyroscopeMeasure(key, m_gyros.at(key).col(bufferSize), m_timeNow))
        {
            yError() << "[FTIMULoggerDevice][run] Unable to get the gyroscope named: " << key;
        }
    }

    // base imu (analog sensor)
    for (const auto& IMU : m_IMUNames)
    {
        if (!m_robotSensorBridge->getIMUMeasurement(IMU, m_analogSensorBuffer, m_timeNow))
        {
            yError() << "[FTIMULoggerDevice][run] Unable to get the imu named : " << IMU;
        }

        // resize the vectors
        m_accelerometers.at(IMU).conservativeResize(3, bufferSize + 1);
        m_orientations.at(IMU).conservativeResize(3, bufferSize + 1);
        m_gyros.at(IMU).conservativeResize(3, bufferSize + 1);

        // it will return a tuple containing the Accelerometer, the gyro and the orientatio
        this->unpackIMU(m_analogSensorBuffer,
                        m_accelerometers.at(IMU).col(bufferSize),
                        m_gyros.at(IMU).col(bufferSize),
                        m_orientations.at(IMU).col(bufferSize));
    }

    // joint state
    m_jointState.at("joint_positions").conservativeResize(m_dofs, bufferSize + 1);
    m_jointState.at("joint_velocities").conservativeResize(m_dofs, bufferSize + 1);
    if (!m_robotSensorBridge->getJointPositions(m_jointState.at("joint_positions").col(bufferSize),
                                                m_timeNow))

    {
        yError() << "[FTIMULoggerDevice][run] Unable to get the joint positions";
    }

    if (!m_robotSensorBridge->getJointVelocities(m_jointState.at("joint_velocities").col(bufferSize),
                                                 m_timeNow))
    {
        yError() << "[FTIMULoggerDevice][run] Unable to get the joint velocities";
    }

    m_time.conservativeResize(bufferSize + 1);
    m_time.row(bufferSize) << m_timeNow;
}

matioCpp::Struct
FTIMULoggerDevice::createStruct(const std::string& structName,
                                const std::unordered_map<std::string, Eigen::MatrixXd>& signal)
{
    std::vector<matioCpp::Variable> vector;
    for (const auto& [key, value] : signal)
    {
        vector.emplace_back(tomatioCpp(value, key));
    }

    return matioCpp::Struct(structName, vector);
}

bool FTIMULoggerDevice::logData()
{
    // set the file name
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::stringstream fileName;
    fileName << "ftimu_dataset_" << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S") << ".mat";

    matioCpp::File file = matioCpp::File::Create(fileName.str());
    auto outTime = Conversions::tomatioCpp(m_time, "time");

    const matioCpp::Struct outFt = this->createStruct("FT", m_fts);
    const matioCpp::Struct outGyros = this->createStruct("Gyros", m_gyros);
    const matioCpp::Struct outAcc = this->createStruct("Accelerometer", m_accelerometers);
    const matioCpp::Struct outOrient = this->createStruct("Orientation", m_orientations);
    matioCpp::Struct outJointState = this->createStruct("Joint_state", m_jointState);
    outJointState.setField(tomatioCpp(m_jointNames, "joints"));

    bool write_ok{true};
    write_ok = write_ok && file.write(outFt);
    write_ok = write_ok && file.write(outGyros);
    write_ok = write_ok && file.write(outAcc);
    write_ok = write_ok && file.write(outOrient);
    write_ok = write_ok && file.write(outJointState);
    write_ok = write_ok && file.write(outTime);

    if (!write_ok)
    {
        yError() << "[FTIMULoggerDevice][logData] Could not write to file.";
        return false;
    }

    return true;
}

bool FTIMULoggerDevice::detachAll()
{
    std::lock_guard<std::mutex> guard(m_deviceMutex);
    if (isRunning())
    {
        stop();
    }

    return true;
}

bool FTIMULoggerDevice::close()
{
    std::lock_guard<std::mutex> guard(m_deviceMutex);
    if (!logData())
    {
        yError() << "[FTIMULoggerDevice][close] Failed to log data.";
    }

    return true;
}
