/**
 * @copyright 2020, 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iomanip>
#include <tuple>

#include <BipedalLocomotion/Conversions/matioCppConversions.h>
#include <BipedalLocomotion/YarpRobotLoggerDevice.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/YarpUtilities/Helper.h>

#include <yarp/os/LogStream.h>

using namespace BipedalLocomotion::YarpUtilities;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::Conversions;
using namespace BipedalLocomotion;

YarpRobotLoggerDevice::YarpRobotLoggerDevice(double period, yarp::os::ShouldUseSystemClock useSystemClock)
    : yarp::os::PeriodicThread(period, useSystemClock)
{
}

YarpRobotLoggerDevice::YarpRobotLoggerDevice()
    : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No)
{
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
    auto bridgeConfig = config.findGroup("RobotSensorBridge");
    if (bridgeConfig.isNull())
    {
        yError() << "[YarpRobotLoggerDevice][setupRobotSensorBridge] Missing required group "
                    "\"RobotSensorBridge\"";
        return false;
    }

    auto originalHandler = std::make_shared<YarpImplementation>();
    originalHandler->set(bridgeConfig);

    m_robotSensorBridge = std::make_unique<YarpSensorBridge>();
    if (!m_robotSensorBridge->initialize(originalHandler))
    {
        yError() << "[YarpRobotLoggerDevice][setupRobotSensorBridge] Could not configure "
                    "RobotSensorBridge";
        return false;
    }

    // Get additional flags required by the device
    if (!originalHandler->getParameter("stream_joint_states", m_streamJointStates))
    {
        yInfo() << "[YarpRobotLoggerDevice][setupRobotSensorBridge] The 'stream_joint_states' "
                   "parameter is not found. The joint states is not logged.";
    }

    if (!originalHandler->getParameter("stream_motor_states", m_streamMotorStates))
    {
        yInfo() << "[YarpRobotLoggerDevice][setupRobotSensorBridge] The 'stream_motor_states' "
                   "parameter is not found. The motor states is not logged.";
    }

    if (!originalHandler->getParameter("stream_motor_PWM", m_streamMotorPWM))
    {
        yInfo() << "[YarpRobotLoggerDevice][setupRobotSensorBridge] The 'stream_motor_PWM' "
                   "parameter is not found. The Motor PWM is not logged.";
    }

    if (!originalHandler->getParameter("stream_pids", m_streamPIDs))
    {
        yInfo() << "[YarpRobotLoggerDevice][setupRobotSensorBridge] The 'stream_pids' parameter is "
                   "not found. The Motor pid values are not logged.";
    }

    if (!originalHandler->getParameter("stream_inertials", m_streamInertials))
    {
        yInfo() << "[YarpRobotLoggerDevice][setupRobotSensorBridge] The 'stream_inertials' parameter is "
                   "not found. The IMU values are not logged.";
    }

    if (!originalHandler->getParameter("stream_cartesian_wrenches", m_streamWrenches))
    {
        yInfo() << "[YarpRobotLoggerDevice][setupRobotSensorBridge] The 'stream_cartesian_wrenches' parameter is "
                   "not found. The cartesian wrench values are not logged.";
    }

    if (!originalHandler->getParameter("stream_forcetorque_sensors", m_streamFTs))
    {
        yInfo() << "[YarpRobotLoggerDevice][setupRobotSensorBridge] The 'stream_forcetorque_sensors' parameter is "
                   "not found. The force/torque sensor values are not logged.";
    }

    return true;
}

bool YarpRobotLoggerDevice::attachAll(const yarp::dev::PolyDriverList& poly)
{
    if (!m_robotSensorBridge->setDriversList(poly))
    {
        yError() << "[YarpRobotLoggerDevice][attachAll] Could not attach drivers list to sensor "
                    "bridge.";
        return false;
    }

    if (!m_robotSensorBridge->getJointsList(m_jointNames))
    {
        yError() << "[YarpRobotLoggerDevice][attachAll] Could not get the joints list.";
        return false;
    }

    if (!m_robotSensorBridge->getSixAxisForceTorqueSensorsList(m_FTNames))
    {
        yError() << "[YarpRobotLoggerDevice][attachAll] Could not get the six axis force torque "
                    "sensors list.";
        return false;
    }

    if (!m_robotSensorBridge->getGyroscopesList(m_gyroNames))
    {
        yError() << "[YarpRobotLoggerDevice][attachAll] Could not get the gyroscope list.";
        return false;
    }

    if (!m_robotSensorBridge->getLinearAccelerometersList(m_accelerometerNames))
    {
        yError() << "[YarpRobotLoggerDevice][attachAll] Could not get the accelerometers list.";
        return false;
    }

    if (!m_robotSensorBridge->getOrientationSensorsList(m_orientationNames))
    {
        yError() << "[YarpRobotLoggerDevice][attachAll] Could not get the orientation sensor list.";
        return false;
    }

    if (!m_robotSensorBridge->getIMUsList(m_IMUNames))
    {
        yError() << "[YarpRobotLoggerDevice][attachAll] Could not get the IMUs list.";
        return false;
    }

    if (!m_robotSensorBridge->getCartesianWrenchesList(m_cartesianWrenchNames))
    {
        yError() << "[YarpRobotLoggerDevice][attachAll] Could not get the cartesian wrenches list ";
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
    // get the data
    if (!m_robotSensorBridge->advance())
    {
        yError() << "[YarpRobotLoggerDevice][run] could not advance sensor bridge.";
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
            yError() << "[YarpRobotLoggerDevice][run] Unable to get the ft named: " << key;
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
            yError() << "[YarpRobotLoggerDevice][run] Unable to get the cartesian wrench named: " << key;
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
            yError() << "[YarpRobotLoggerDevice][run] Unable to get the orientation named: " << key;
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
            yError() << "[YarpRobotLoggerDevice][run] Unable to get the accelerometer named: " << key;
        }
    }

    for (const auto& key : m_gyroNames)
    {
        m_gyros.at(key).conservativeResize(3, bufferSize + 1);
        if (!m_robotSensorBridge->getGyroscopeMeasure(key, m_gyros.at(key).col(bufferSize), m_timeNow))
        {
            yError() << "[YarpRobotLoggerDevice][run] Unable to get the gyroscope named: " << key;
        }
    }

    // base imu (analog sensor)

    for (const auto& IMU : m_IMUNames)
    {
        if (!m_robotSensorBridge->getIMUMeasurement(IMU, m_analogSensorBuffer, m_timeNow))
        {
            yError() << "[YarpRobotLoggerDevice][run] Unable to get the imu named : " << IMU;
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
            yError() << "[YarpRobotLoggerDevice][run] Unable to get the joint positions";
        }

        if (!m_robotSensorBridge
                 ->getJointVelocities(m_jointState.at("joint_velocities").col(bufferSize),
                                      m_timeNow))
        {
            yError() << "[YarpRobotLoggerDevice][run] Unable to get the joint velocities";
        }

        if (!m_robotSensorBridge->getJointTorques(m_jointState.at("joint_torques").col(bufferSize),
                                                  m_timeNow))
        {
            yError() << "[YarpRobotLoggerDevice][run] Unable to get the joint torques";
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
            yError() << "[YarpRobotLoggerDevice][run] Unable to get the motor positions";
        }

        if (!m_robotSensorBridge
                 ->getMotorVelocities(m_motorState.at("motor_velocities").col(bufferSize),
                                      m_timeNow))
        {
            yError() << "[YarpRobotLoggerDevice][run] Unable to get the motor velocities";
        }

        if (!m_robotSensorBridge->getMotorCurrents(m_motorState.at("motor_currents").col(bufferSize),
                                                   m_timeNow))
        {
            yError() << "[YarpRobotLoggerDevice][run] Unable to get the motor currents";
        }
    }

    // Motor PWM
    if (m_streamMotorPWM)
    {
        m_motorPWMs.at("PWM").conservativeResize(m_dofs, bufferSize + 1);
        if (!m_robotSensorBridge->getMotorPWMs(m_motorPWMs.at("PWM").col(bufferSize), m_timeNow))
        {
            yError() << "[YarpRobotLoggerDevice][run] Unable to get the motor PWMs";
        }
    }

    if (m_streamPIDs)
    {
        m_PIDs.at("PID").conservativeResize(m_dofs, bufferSize + 1);
        if (!m_robotSensorBridge->getPidPositions(m_PIDs.at("PID").col(bufferSize), m_timeNow))
        {
            yError() << "[YarpRobotLoggerDevice][run] Unable to get the PIDs references";
        }
    }

    m_time.conservativeResize(bufferSize + 1);
    m_time.row(bufferSize) << m_timeNow;
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
        yError() << "[YarpRobotLoggerDevice][logData] Could not write to file.";
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
        yError() << "[YarpRobotLoggerDevice][close] Failed to log data.";
    }

    return true;
}
