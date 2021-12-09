/**
 * @copyright 2020, 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iomanip>
#include <tuple>

#include <BipedalLocomotion/Conversions/matioCppConversions.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/TextLogging/LoggerBuilder.h>
#include <BipedalLocomotion/TextLogging/YarpLogger.h>
#include <BipedalLocomotion/YarpRobotLoggerDevice.h>
#include <BipedalLocomotion/YarpUtilities/Helper.h>

using namespace BipedalLocomotion::YarpUtilities;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::Conversions;
using namespace BipedalLocomotion;

YarpRobotLoggerDevice::YarpRobotLoggerDevice(double period, yarp::os::ShouldUseSystemClock useSystemClock)
    : yarp::os::PeriodicThread(period, useSystemClock)
{
    // Use the yarp clock in blf
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());

    // the logging message are streamed using yarp
    BipedalLocomotion::TextLogging::LoggerBuilder::setFactory(
        std::make_shared<BipedalLocomotion::TextLogging::YarpLoggerFactory>());
}

YarpRobotLoggerDevice::YarpRobotLoggerDevice()
    : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No)
{
    // Use the yarp clock in blf
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());

    // the logging message are streamed using yarp
    BipedalLocomotion::TextLogging::LoggerBuilder::setFactory(
        std::make_shared<BipedalLocomotion::TextLogging::YarpLoggerFactory>());
}

YarpRobotLoggerDevice::~YarpRobotLoggerDevice() = default;

bool YarpRobotLoggerDevice::open(yarp::os::Searchable& config)
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

bool YarpRobotLoggerDevice::setupRobotSensorBridge(yarp::os::Searchable& config)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice][setupRobotSensorBridge]";

    auto bridgeConfig = config.findGroup("RobotSensorBridge");
    if (bridgeConfig.isNull())
    {
        log()->error("{} Missing required group 'RobotSensorBridge'", logPrefix);
        return false;
    }

    auto originalHandler = std::make_shared<YarpImplementation>();
    originalHandler->set(bridgeConfig);

    m_robotSensorBridge = std::make_unique<YarpSensorBridge>();
    if (!m_robotSensorBridge->initialize(originalHandler))
    {
        log()->error("{} Unable to configure the 'SensorBridge'", logPrefix);
        return false;
    }

    // Get additional flags required by the device
    if (!originalHandler->getParameter("stream_joint_states", m_streamJointStates))
    {
        log()->info("{} The 'stream_joint_states' parameter is not found. The joint states is not "
                    "logged",
                    logPrefix);
    }

    if (!originalHandler->getParameter("stream_motor_states", m_streamMotorStates))
    {
        log()->info("{} The 'stream_motor_states' parameter is not found. The motor states is not "
                    "logged",
                    logPrefix);
    }

    if (!originalHandler->getParameter("stream_motor_PWM", m_streamMotorPWM))
    {
        log()->info("{} The 'stream_motor_PWM' parameter is not found. The motor PWM is not logged",
                    logPrefix);
    }

    if (!originalHandler->getParameter("stream_pids", m_streamPIDs))
    {
        log()->info("{} The 'stream_pids' parameter is not found. The motor pid values are not "
                    "logged",
                    logPrefix);
    }

    return true;
}

bool YarpRobotLoggerDevice::attachAll(const yarp::dev::PolyDriverList& poly)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice][attachAll]";

    if (!m_robotSensorBridge->setDriversList(poly))
    {
        log()->error("{} Could not attach drivers list to sensor bridge.", logPrefix);
        return false;
    }

    if (!m_robotSensorBridge->getJointsList(m_jointNames))
    {
        log()->error("{} Could not get the joints list.", logPrefix);
        return false;
    }

    if (!m_robotSensorBridge->getSixAxisForceTorqueSensorsList(m_FTNames))
    {
        log()->error("{} Could not get the six axis force torque sensors list.", logPrefix);
        return false;
    }

    if (!m_robotSensorBridge->getGyroscopesList(m_gyroNames))
    {
        log()->error("{} Could not get the gyroscope list.", logPrefix);
        return false;
    }

    if (!m_robotSensorBridge->getLinearAccelerometersList(m_accelerometerNames))
    {
        log()->error("{} Could not get the accelerometers list.", logPrefix);
        return false;
    }

    if (!m_robotSensorBridge->getOrientationSensorsList(m_orientationNames))
    {
        log()->error("{} Could not get the orientation sensor list.", logPrefix);
        return false;
    }

    if (!m_robotSensorBridge->getIMUsList(m_IMUNames))
    {
        log()->error("{} Could not get the IMUs list.", logPrefix);
        return false;
    }

    if (!m_robotSensorBridge->getCartesianWrenchesList(m_cartesianWrenchNames))
    {
        log()->error("{} Could not get the cartesian wrenches list.", logPrefix);
        return false;
    }

    // prepare the unordered_maps
    for (const auto& ft : m_FTNames)
    {
        m_fts[ft] = Eigen::MatrixXd();
    }

    for (const auto& wrench : m_cartesianWrenchNames)
    {
        m_wrenches[wrench] = Eigen::MatrixXd();
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

    // an IMU consists gyro acc and orient
    for (const auto& IMU : m_IMUNames)
    {
        m_gyros[IMU] = Eigen::MatrixXd();
        m_accelerometers[IMU] = Eigen::MatrixXd();
        m_orientations[IMU] = Eigen::MatrixXd();
    }

    m_dofs = m_jointNames.size();

    m_jointState["joint_positions"] = Eigen::MatrixXd();
    m_jointState["joint_velocities"] = Eigen::MatrixXd();
    m_jointState["joint_torques"] = Eigen::MatrixXd();

    m_motorState["motor_positions"] = Eigen::MatrixXd();
    m_motorState["motor_velocities"] = Eigen::MatrixXd();
    m_motorState["motor_currents"] = Eigen::MatrixXd();

    m_motorPWMs["PWM"] = Eigen::MatrixXd();

    m_PIDs["PID"] = Eigen::MatrixXd();

    start();
    return true;
}

void YarpRobotLoggerDevice::unpackIMU(Eigen::Ref<const analog_sensor_t> signal,
                                  Eigen::Ref<accelerometer_t> accelerometer,
                                  Eigen::Ref<gyro_t> gyro,
                                  Eigen::Ref<orientation_t> orientation)
{
    // the output consists 12 double, organized as follows:
    //  euler angles [3]
    // linear acceleration [3]
    // angular speed [3]
    // magnetic field [3]
    // http://wiki.icub.org/wiki/Inertial_Sensor
    orientation = signal.segment<3>(0);
    accelerometer = signal.segment<3>(3);
    gyro = signal.segment<3>(6);
}

void YarpRobotLoggerDevice::run()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice][run]";

    // get the data
    if (!m_robotSensorBridge->advance())
    {
        log()->error("{} Could not advance sensor bridge.", logPrefix);
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
            log()->error("{} Unable to get the ft named: {}.", logPrefix, key);
        }
    }

    // get the cartesian wrenches
    for (const auto& key : m_cartesianWrenchNames)
    {
        m_wrenches.at(key).conservativeResize(6, bufferSize + 1);
        if (!m_robotSensorBridge->getCartesianWrench(key,
                                                     m_wrenches.at(key).col(bufferSize),
                                                     m_timeNow))
        {
            log()->error("{} Unable to get the Cartesian wrench named: {}.", logPrefix, key);
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
            log()->error("{} Unable to get the orientation named: {}.", logPrefix, key);
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
            log()->error("{} Unable to get the accelerometer named: {}.", logPrefix, key);
        }
    }

    for (const auto& key : m_gyroNames)
    {
        m_gyros.at(key).conservativeResize(3, bufferSize + 1);
        if (!m_robotSensorBridge->getGyroscopeMeasure(key,
                                                      m_gyros.at(key).col(bufferSize),
                                                      m_timeNow))
        {
            log()->error("{} Unable to get the gyroscope named: {}.", logPrefix, key);
        }
    }

    // base imu (analog sensor)

    for (const auto& IMU : m_IMUNames)
    {
        if (!m_robotSensorBridge->getIMUMeasurement(IMU, m_analogSensorBuffer, m_timeNow))
        {
            log()->error("{} Unable to get the imu named: {}.", logPrefix, IMU);
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
    if (m_streamJointStates)
    {
        m_jointState.at("joint_positions").conservativeResize(m_dofs, bufferSize + 1);
        m_jointState.at("joint_velocities").conservativeResize(m_dofs, bufferSize + 1);
        m_jointState.at("joint_torques").conservativeResize(m_dofs, bufferSize + 1);
        if (!m_robotSensorBridge
                 ->getJointPositions(m_jointState.at("joint_positions").col(bufferSize), m_timeNow))

        {
            log()->error("{} Unable to get the joint positions.", logPrefix);
        }

        if (!m_robotSensorBridge
                 ->getJointVelocities(m_jointState.at("joint_velocities").col(bufferSize),
                                      m_timeNow))
        {
            log()->error("{} Unable to get the joint velocities.", logPrefix);
        }

        if (!m_robotSensorBridge->getJointTorques(m_jointState.at("joint_torques").col(bufferSize),
                                                  m_timeNow))
        {
            log()->error("{} Unable to get the joint torques.", logPrefix);
        }
    }

    // Motor state
    if (m_streamMotorStates)
    {
        m_motorState.at("motor_positions").conservativeResize(m_dofs, bufferSize + 1);
        m_motorState.at("motor_velocities").conservativeResize(m_dofs, bufferSize + 1);
        m_motorState.at("motor_currents").conservativeResize(m_dofs, bufferSize + 1);
        if (!m_robotSensorBridge
                 ->getMotorPositions(m_motorState.at("motor_positions").col(bufferSize), m_timeNow))

        {
            log()->error("{} Unable to get the motor positions.", logPrefix);
        }

        if (!m_robotSensorBridge
                 ->getMotorVelocities(m_motorState.at("motor_velocities").col(bufferSize),
                                      m_timeNow))
        {
            log()->error("{} Unable to get the motor velocities.", logPrefix);
        }

        if (!m_robotSensorBridge->getMotorCurrents(m_motorState.at("motor_currents").col(bufferSize),
                                                   m_timeNow))
        {
            log()->error("{} Unable to get the motor currents.", logPrefix);
        }
    }

    // Motor PWM
    if (m_streamMotorPWM)
    {
        m_motorPWMs.at("PWM").conservativeResize(m_dofs, bufferSize + 1);
        if (!m_robotSensorBridge->getMotorPWMs(m_motorPWMs.at("PWM").col(bufferSize), m_timeNow))
        {
            log()->error("{} Unable to get the motor PWMs.", logPrefix);
        }
    }

    if (m_streamPIDs)
    {
        m_PIDs.at("PID").conservativeResize(m_dofs, bufferSize + 1);
        if (!m_robotSensorBridge->getPidPositions(m_PIDs.at("PID").col(bufferSize), m_timeNow))
        {
            log()->error("{} Unable to get the PIDs references.", logPrefix);
        }
    }

    m_time.conservativeResize(bufferSize + 1);
    m_time.row(bufferSize) << BipedalLocomotion::clock().now().count();
}

matioCpp::Struct
YarpRobotLoggerDevice::createStruct(const std::string& structName,
                                const std::unordered_map<std::string, Eigen::MatrixXd>& signal)
{
    std::vector<matioCpp::Variable> vector;
    for (const auto& [key, value] : signal)
    {
        vector.emplace_back(tomatioCpp(value, key));
    }

    return matioCpp::Struct(structName, vector);
}

bool YarpRobotLoggerDevice::logData()
{
    // set the file name
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::stringstream fileName;
    fileName << "yarprobot_dataset_" << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S") << ".mat";

    matioCpp::File file = matioCpp::File::Create(fileName.str());
    auto outTime = Conversions::tomatioCpp(m_time, "time");

    const matioCpp::Struct outFt = this->createStruct("FT", m_fts);
    const matioCpp::Struct outWrench = this->createStruct("CartesianWrench", m_wrenches);
    const matioCpp::Struct outGyros = this->createStruct("Gyros", m_gyros);
    const matioCpp::Struct outAcc = this->createStruct("Accelerometer", m_accelerometers);
    const matioCpp::Struct outOrient = this->createStruct("Orientation", m_orientations);
    matioCpp::Struct outJointState = this->createStruct("Joint_state", m_jointState);
    matioCpp::Struct outMotorState = this->createStruct("Motor_state", m_motorState);
    const matioCpp::Struct outPWM = this->createStruct("Motor_PWM", m_motorPWMs);
    const matioCpp::Struct outPID = this->createStruct("PID", m_PIDs);
    outJointState.setField(tomatioCpp(m_jointNames, "joints"));

    bool write_ok{true};
    write_ok = write_ok && file.write(outFt);
    write_ok = write_ok && file.write(outWrench);
    write_ok = write_ok && file.write(outGyros);
    write_ok = write_ok && file.write(outAcc);
    write_ok = write_ok && file.write(outOrient);
    write_ok = write_ok && file.write(outJointState);
    write_ok = write_ok && file.write(outMotorState);
    write_ok = write_ok && file.write(outPWM);
    write_ok = write_ok && file.write(outPID);
    write_ok = write_ok && file.write(outTime);

    if (!write_ok)
    {
        log()->error("[YarpRobotLoggerDevice][logData] Could not write to file.");
        return false;
    }

    return true;
}

bool YarpRobotLoggerDevice::detachAll()
{
    std::lock_guard<std::mutex> guard(m_deviceMutex);
    if (isRunning())
    {
        stop();
    }

    return true;
}

bool YarpRobotLoggerDevice::close()
{
    std::lock_guard<std::mutex> guard(m_deviceMutex);
    if (!logData())
    {
        log()->error("[YarpRobotLoggerDevice][close] Failed to log the data.");
    }

    return true;
}
