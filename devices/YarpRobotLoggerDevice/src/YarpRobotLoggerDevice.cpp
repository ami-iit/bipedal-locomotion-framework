/**
 * @copyright 2020, 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <string>
#include <tuple>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/TextLogging/LoggerBuilder.h>
#include <BipedalLocomotion/TextLogging/YarpLogger.h>
#include <BipedalLocomotion/YarpUtilities/Helper.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollection.h>

#include <BipedalLocomotion/YarpRobotLoggerDevice.h>
#include <BipedalLocomotion/YarpTextLoggingUtilities.h>


#include <yarp/os/BufferedPort.h>
#include <yarp/profiler/NetworkProfiler.h>

#include <yarp/telemetry/experimental/BufferConfig.h>
#include <yarp/telemetry/experimental/BufferManager.h>

#include <iostream>
#include <fstream>
#include <cstdio>


using namespace BipedalLocomotion::YarpUtilities;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion;

VISITABLE_STRUCT(TextLoggingEntry,
                 level,
                 text,
                 filename,
                 line,
                 function,
                 hostname,
                 cmd,
                 args,
                 pid,
                 thread_id,
                 component,
                 id,
                 systemtime,
                 networktime,
                 externaltime,
                 backtrace,
                 yarprun_timestamp,
                 local_timestamp);

void findAndReplaceAll(std::string& data, std::string toSearch, std::string replaceStr)
{
    // Get the first occurrence
    size_t pos = data.find(toSearch);
    // Repeat till end is reached
    while (pos != std::string::npos)
    {
        // Replace this occurrence of Sub String
        data.replace(pos, toSearch.size(), replaceStr);
        // Get the next occurrence from the current position
        pos = data.find(toSearch, pos + replaceStr.size());
    }
}

YarpRobotLoggerDevice::YarpRobotLoggerDevice(double period,
                                             yarp::os::ShouldUseSystemClock useSystemClock)
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
    auto params = std::make_shared<ParametersHandler::YarpImplementation>(config);

    double devicePeriod{0.01};
    if (params->getParameter("sampling_period_in_s", devicePeriod))
    {
        this->setPeriod(devicePeriod);
    }

    if (!this->setupRobotSensorBridge(params->getGroup("RobotSensorBridge")))
    {
        return false;
    }

    if (this->setupRobotCameraBridge(params->getGroup("RobotCameraBridge")))
    {
        // Currently the logger supports only rgb cameras
        if (m_cameraBridge->getMetaData().bridgeOptions.isRGBCameraEnabled)
            // || m_cameraBridge->getMetaData().bridgeOptions.isRGBDCameraEnabled)
        {
            std::vector<int> rgbFPS;
            if (!params->getParameter("rgb_cameras_fps", rgbFPS))
            {
                return false;
            }

            const auto& rgbCameras = m_cameraBridge->getMetaData().sensorsList.rgbCamerasList;
            if (rgbFPS.size() != rgbCameras.size())
            {
                log()->error("[YarpRobotLoggerDevice::open] Mismatch between the number of cameras "
                             "and the vector containg the FPS. Number of cameras: {}. Size of the "
                             "FPS vector {}.",
                             rgbCameras.size(),
                             rgbFPS.size());
                return false;
            }

            for (unsigned int i = 0; i < rgbFPS.size(); i++)
            {
                if (rgbFPS[i] <= 0)
                {
                    log()->error("[YarpRobotLoggerDevice::open] The FPS associated to the camera "
                                 "{} is negative or equal to zero.",
                                 i);
                    return false;
                }

                // get the desired fps for each camera
                m_videoWriters[rgbCameras[i]].fps = rgbFPS[i];
            }
        }

        if (m_cameraBridge->getMetaData().bridgeOptions.isRGBDCameraEnabled)
        {
            log()->warn("[YarpRobotLoggerDevice::open] The logger does not support rgbd cameras.");
        }

    } else
    {
        log()->info("[YarpRobotLoggerDevice::open] The video will not be recorded");
    }

    if (!this->setupTelemetry(params->getGroup("Telemetry"), devicePeriod))
    {
        return false;
    }

    if (!this->setupExogenousInputs(params->getGroup("ExogenousSignals")))
    {
        return false;
    }

    return true;
}

bool YarpRobotLoggerDevice::setupExogenousInputs(
    std::weak_ptr<const ParametersHandler::IParametersHandler> params)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::setupExogenousInputs]";

    auto ptr = params.lock();
    if (ptr == nullptr)
    {
        log()->info("{} No exogenous input will be logged.", logPrefix);
        return true;
    }

    std::vector<std::string> inputs;
    if (!ptr->getParameter("exogenous_inputs", inputs))
    {
        log()->error("{} Unable to get the exogenous inputs.", logPrefix);
        return false;
    }

    for (const auto& input : inputs)
    {
        auto group = ptr->getGroup(input).lock();
        std::string portName, signalName;
        if (group == nullptr || !group->getParameter("port_name", portName)
            || !group->getParameter("signal_name", signalName))
        {
            log()->error("{} Unable to get the parameters related to the input: {}.",
                         logPrefix,
                         input);
            return false;
        }

        if (!m_exogenousPorts[signalName].open(portName))
        {
            log()->error("{} Unable to open the port named: {}.", logPrefix, portName);
            return false;
        }
    }

    return true;
}

bool YarpRobotLoggerDevice::setupTelemetry(
    std::weak_ptr<const ParametersHandler::IParametersHandler> params, const double& devicePeriod)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::setupTelemetry]";

    auto ptr = params.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameters handler is not valid.", logPrefix);
        return false;
    }

    yarp::telemetry::experimental::BufferConfig config;
    config.yarp_robot_name = std::getenv("YARP_ROBOT_NAME");
    config.filename = "robot_logger_device";
    config.auto_save = true;
    config.save_periodically = true;
    config.file_indexing = "%Y_%m_%d_%H_%M_%S";
    config.mat_file_version = matioCpp::FileVersion::MAT7_3;

    if (!ptr->getParameter("save_period", config.save_period))
    {
        log()->error("{} Unable to get the 'save_period' parameter for the telemetry.", logPrefix);
        return false;
    }

    // the telemetry will flush the content of its storage every config.save_period
    // and this device runs every devicePeriod
    // so the size of the telemetry buffer must be at least config.save_period / devicePeriod
    // to be sure we are not going to lose data the buffer will be 10% longer
    constexpr double percentage = 0.1;
    config.n_samples = static_cast<int>(std::ceil((1 + percentage) //
                                                  * (config.save_period / devicePeriod)));

    return m_bufferManager.configure(config);
}

bool YarpRobotLoggerDevice::setupRobotSensorBridge(
    std::weak_ptr<const ParametersHandler::IParametersHandler> params)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::setupRobotSensorBridge]";

    auto ptr = params.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameters handler is not valid.", logPrefix);
        return false;
    }

    m_robotSensorBridge = std::make_unique<YarpSensorBridge>();
    if (!m_robotSensorBridge->initialize(ptr))
    {
        log()->error("{} Unable to configure the 'SensorBridge'", logPrefix);
        return false;
    }

    // Get additional flags required by the device
    if (!ptr->getParameter("stream_joint_states", m_streamJointStates))
    {
        log()->info("{} The 'stream_joint_states' parameter is not found. The joint states is not "
                    "logged",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_motor_states", m_streamMotorStates))
    {
        log()->info("{} The 'stream_motor_states' parameter is not found. The motor states is not "
                    "logged",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_motor_PWM", m_streamMotorPWM))
    {
        log()->info("{} The 'stream_motor_PWM' parameter is not found. The motor PWM is not logged",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_pids", m_streamPIDs))
    {
        log()->info("{} The 'stream_pids' parameter is not found. The motor pid values are not "
                    "logged",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_inertials", m_streamInertials))
    {
        log()->info("{} The 'stream_inertials' parameter is not found. The IMU values are not "
                    "logged",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_cartesian_wrenches", m_streamCartesianWrenches))
    {
        log()->info("{} The 'stream_cartesian_wrenches' parameter is not found. The cartesian "
                    "wrench values are not "
                    "logged",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_forcetorque_sensors", m_streamFTSensors))
    {
        log()->info("{} The 'stream_forcetorque_sensors' parameter is not found. The FT values are "
                    "not "
                    "logged",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_temperatures", m_streamTemperatureSensors))
    {
        log()->info("{} The 'stream_temperatures' parameter is not found. The temperature sensor "
                    "values are not "
                    "logged",
                    logPrefix);
    }

    return true;
}

bool YarpRobotLoggerDevice::setupRobotCameraBridge(
    std::weak_ptr<const ParametersHandler::IParametersHandler> params)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::setupRobotCameraBridge]";

    auto ptr = params.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameters handler is not valid.", logPrefix);
        return false;
    }

    m_cameraBridge = std::make_unique<YarpCameraBridge>();
    if (!m_cameraBridge->initialize(ptr))
    {
        log()->error("{} Unable to configure the 'Camera bridge'", logPrefix);
        return false;
    }

    return true;
}


bool YarpRobotLoggerDevice::attachAll(const yarp::dev::PolyDriverList& poly)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::attachAll]";

    if (!m_robotSensorBridge->setDriversList(poly))
    {
        log()->error("{} Could not attach drivers list to sensor bridge.", logPrefix);
        return false;
    }

    // The user can avoid to record the camera
    if (m_cameraBridge != nullptr)
    {
        if (!m_cameraBridge->setDriversList(poly))
        {
            log()->error("{} Could not attach drivers list to camera bridge.", logPrefix);
            return false;
        }
    }

    // TODO this should be removed
    // this sleep is required since the sensor bridge could be not ready
    using namespace std::chrono_literals;
    BipedalLocomotion::clock().sleepFor(2000ms);

    std::vector<std::string> joints;
    if (!m_robotSensorBridge->getJointsList(joints))
    {
        log()->error("{} Could not get the joints list.", logPrefix);
        return false;
    }

    const unsigned dofs = joints.size();
    m_bufferManager.setDescriptionList(joints);

    bool ok = true;

    // prepare the telemetry
    if (m_streamJointStates)
    {
        ok = ok && m_bufferManager.addChannel({"joints_state::positions", {dofs, 1}, joints});
        ok = ok && m_bufferManager.addChannel({"joints_state::velocities", {dofs, 1}, joints});
        ok = ok && m_bufferManager.addChannel({"joints_state::accelerations", {dofs, 1}, joints});
        ok = ok && m_bufferManager.addChannel({"joints_state::torques", {dofs, 1}, joints});
    }
    if (m_streamMotorStates)
    {
        ok = ok && m_bufferManager.addChannel({"motors_state::positions", {dofs, 1}, joints});
        ok = ok && m_bufferManager.addChannel({"motors_state::velocities", {dofs, 1}, joints});
        ok = ok && m_bufferManager.addChannel({"motors_state::accelerations", {dofs, 1}, joints});
        ok = ok && m_bufferManager.addChannel({"motors_state::currents", {dofs, 1}, joints});
    }

    if (m_streamMotorPWM)
    {
        ok = ok && m_bufferManager.addChannel({"motors_state::PWM", {dofs, 1}, joints});
    }

    if (m_streamPIDs)
    {
        ok = ok && m_bufferManager.addChannel({"PIDs", {dofs, 1}, joints});
    }

    if (m_streamFTSensors)
    {
        for (const auto& sensorName : m_robotSensorBridge->getSixAxisForceTorqueSensorsList())
        {
            ok = ok
                 && m_bufferManager.addChannel({"FTs::" + sensorName,
                                                {6, 1}, //
                                                {"f_x", "f_y", "f_z", "mu_x", "mu_y", "mu_z"}});
        }
    }

    if (m_streamInertials)
    {
        for (const auto& sensorName : m_robotSensorBridge->getGyroscopesList())
        {
            ok = ok
                 && m_bufferManager.addChannel({"gyros::" + sensorName,
                                                {3, 1}, //
                                                {"omega_x", "omega_y", "omega_z"}});
        }

        for (const auto& sensorName : m_robotSensorBridge->getLinearAccelerometersList())
        {
            ok = ok
                 && m_bufferManager.addChannel({"accelerometers::" + sensorName,
                                                {3, 1}, //
                                                {"a_x", "a_y", "a_z"}});
        }

        for (const auto& sensorName : m_robotSensorBridge->getOrientationSensorsList())
        {
            ok = ok
                 && m_bufferManager.addChannel({"orientations::" + sensorName,
                                                {3, 1}, //
                                                {"r", "p", "y"}});
        }

        // an IMU contains a gyro accelerometer and an orientation sensor
        for (const auto& sensorName : m_robotSensorBridge->getIMUsList())
        {
            ok = ok
                 && m_bufferManager.addChannel({"accelerometers::" + sensorName,
                                                {3, 1}, //
                                                {"a_x", "a_y", "a_z"}})
                 && m_bufferManager.addChannel({"gyros::" + sensorName,
                                                {3, 1}, //
                                                {"omega_x", "omega_y", "omega_z"}})
                 && m_bufferManager.addChannel({"orientations::" + sensorName,
                                                {3, 1}, //
                                                {"r", "p", "y"}});
        }
    }

    if (m_streamCartesianWrenches)
    {
        for (const auto& sensorName : m_robotSensorBridge->getCartesianWrenchesList())
        {
            ok = ok
                 && m_bufferManager.addChannel({"cartesian_wrenches::" + sensorName,
                                                {6, 1}, //
                                                {"f_x", "f_y", "f_z", "mu_x", "mu_y", "mu_z"}});
        }
    }

    if (m_streamTemperatureSensors)
    {
        for (const auto& sensorName : m_robotSensorBridge->getTemperatureSensorsList())
        {
            ok = ok
                 && m_bufferManager.addChannel({"temperatures::" + sensorName,
                                                {1, 1}, //
                                                {"temperature"}});
        }
    }

    // resize the temporary vectors
    m_jointSensorBuffer.resize(dofs);

    // open the TextLogging port
    ok = ok && m_textLoggingPort.open(m_textLoggingPortName);
    // run the thread
    m_lookForNewLogsThread = std::thread([this] { this->lookForNewLogs(); });


    // The user can avoid to record the camera
    if (m_cameraBridge != nullptr)
    {
        ok = ok && m_cameraBridge->getRGBCamerasList(m_rgbCamerasList);

        for (const auto& camera : m_rgbCamerasList)
        {
            const auto& cameraInfo
                = m_cameraBridge->getMetaData().bridgeOptions.rgbImgDimensions.find(camera);
            if (cameraInfo == m_cameraBridge->getMetaData().bridgeOptions.rgbImgDimensions.end())
            {
                log()->error("{} Unable to get the info of the camera named {}.",
                             logPrefix,
                             camera);
                return false;
            }

            m_videoWriters[camera].writer
                = std::make_shared<cv::VideoWriter>("output_" + camera + ".mp4",
                                                    cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                                                    m_videoWriters[camera].fps,
                                                    cv::Size(cameraInfo->second.first,
                                                             cameraInfo->second.second));
        }

        // if there is at least one camera we set the callback
        if (m_rgbCamerasList.size() != 0)
        {
            ok = ok
                 && m_bufferManager.setSaveCallback(
                     [this](const std::string& filePrefix,
                            const yarp::telemetry::experimental::SaveCallbackSaveMethod& method)
                         -> bool { return this->saveVideo(filePrefix, method); });
        }

        if (ok)
        {
            for (auto& [cameraName, writer] : m_videoWriters)
            {
                // start a separate the thread for each camera
                writer.videoThread = std::thread([&] { this->recordVideo(cameraName, writer); });
            }
        }
    }

    if (ok)
    {
        return start();
    }

    return ok;
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

void YarpRobotLoggerDevice::lookForNewLogs()
{
    yarp::profiler::NetworkProfiler::ports_name_set yarpPorts;
    constexpr auto textLoggingPortPrefix = "/log/";

    auto time = BipedalLocomotion::clock().now();
    auto oldTime = time;
    auto wakeUpTime = time;
    const auto lookForNewLogsPeriod = std::chrono::duration<double>(0.5);
    m_lookForNewLogsIsRunning = true;

    while (m_lookForNewLogsIsRunning)
    {
        // detect if a clock has been reset
        oldTime = time;
        time = BipedalLocomotion::clock().now();
        // if the current time is lower than old time, the timer has been reset.
        if ((time - oldTime).count() < 1e-12)
        {
            wakeUpTime = time;
        }
        wakeUpTime += lookForNewLogsPeriod;

        // check for new messages
        yarp::profiler::NetworkProfiler::getPortsList(yarpPorts);
        for (const auto& port : yarpPorts)
        {
            // check if the port exist is a logging port
            if ((port.name.rfind(textLoggingPortPrefix, 0) == 0)
                && (m_textLoggingPortNames.find(port.name) == m_textLoggingPortNames.end()))
            {
                m_textLoggingPortNames.insert(port.name);
                yarp::os::Network::connect(port.name, m_textLoggingPortName);
            }
        }

        // release the CPU
        BipedalLocomotion::clock().yield();

        // sleep
        BipedalLocomotion::clock().sleepUntil(wakeUpTime);
    }
}

// void YarpRobotLoggerDevice::saveTextLogging()
// {
//     auto time = BipedalLocomotion::clock().now();
//     auto oldTime = time;
//     auto wakeUpTime = time;
//     const auto period = std::chrono::duration<double>(0.05);
//     m_saveTextLoggingIsRunning = true;

//     while (m_saveTextLoggingIsRunning)
//     {
//         // detect if a clock has been reset
//         oldTime = time;
//         time = BipedalLocomotion::clock().now();
//         // if the current time is lower than old time, the timer has been reset.
//         if ((time - oldTime).count() < 1e-12)
//         {
//             wakeUpTime = time;
//         }
//         wakeUpTime += period;

//         {
//             std::lock_guard lock(m_newTextLoggingPortsMutex);
//             for (const auto& portName : m_newTextLoggingPorts)
//             {

//             }
//             m_newTextLoggingPorts.clear();
//         }
//         // check for new messages
//         yarp::profiler::NetworkProfiler::getPortsList(yarpPorts);
//         for (const auto& port : yarpPorts)
//         {
//             // check if the port exist is a logging port
//             if ((port.name.rfind(textLoggingPortPrefix, 0) == 0)
//                 && (m_textLoggingPortNames.find(port.name) != m_textLoggingPortNames.end()))
//             {
//                 m_textLoggingPortNames.insert(port.name);
//                 yarp::os::Network::connect(port.name, m_textLoggingPortName);

//                 std::lock_guard lock(m_newTextLoggingPortsMutex);
//                 m_newTextLoggingPorts.insert(port.name);
//             }
//         }

//         // release the CPU
//         BipedalLocomotion::clock().yield();

//         // sleep
//         BipedalLocomotion::clock().sleepUntil(wakeUpTime);
//     }
// }

void YarpRobotLoggerDevice::recordVideo(const std::string& cameraName, VideoWriter& writer)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::recordVideo]";

    auto time = BipedalLocomotion::clock().now();
    auto oldTime = time;
    auto wakeUpTime = time;
    writer.recordVideoIsRunning = true;
    const auto recordVideoPeriod = std::chrono::duration<double>(1 / double(writer.fps));

    while (writer.recordVideoIsRunning)
    {
        // detect if a clock has been reset
        oldTime = time;
        time = BipedalLocomotion::clock().now();
        // if the current time is lower than old time, the timer has been reset.
        if ((time - oldTime).count() < 1e-12)
        {
            wakeUpTime = time;
        }
        wakeUpTime += recordVideoPeriod;

        // get the frame from the camera
        if (!m_cameraBridge->getColorImage(cameraName, writer.frame))
        {
            log()->info("{} Unable to get the frame of the camera named: {}. The previous frame "
                        "will be used.",
                        logPrefix,
                        cameraName);
        }

        // save the frame in the video writer
        {
            std::lock_guard<std::mutex> lock(writer.mutex);
            writer.writer->write(writer.frame);
        }

        // release the CPU
        BipedalLocomotion::clock().yield();

        if (wakeUpTime < BipedalLocomotion::clock().now())
        {
            log()->info("{} The video thread spent more time than expected to save the camera "
                        "named: {}.",
                        logPrefix,
                        cameraName);
        }

        // sleep
        BipedalLocomotion::clock().sleepUntil(wakeUpTime);
    }
}

void YarpRobotLoggerDevice::run()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::run]";

    // get the data
    if (!m_robotSensorBridge->advance())
    {
        log()->error("{} Could not advance sensor bridge.", logPrefix);
    }

    const double time = BipedalLocomotion::clock().now().count();

    // collect the data
    if (m_streamJointStates)
    {
        if (m_robotSensorBridge->getJointPositions(m_jointSensorBuffer))
        {
            m_bufferManager.push_back(m_jointSensorBuffer, time, "joints_state::positions");
        }
        if (m_robotSensorBridge->getJointVelocities(m_jointSensorBuffer))
        {
            m_bufferManager.push_back(m_jointSensorBuffer, time, "joints_state::velocities");
        }
        if (m_robotSensorBridge->getJointAccelerations(m_jointSensorBuffer))
        {
            m_bufferManager.push_back(m_jointSensorBuffer, time, "joints_state::accelerations");
        }
        if (m_robotSensorBridge->getJointTorques(m_jointSensorBuffer))
        {
            m_bufferManager.push_back(m_jointSensorBuffer, time, "joints_state::torques");
        }
    }

    if (m_streamMotorStates)
    {
        if (m_robotSensorBridge->getMotorPositions(m_jointSensorBuffer))
        {
            m_bufferManager.push_back(m_jointSensorBuffer, time, "motors_state::positions");
        }
        if (m_robotSensorBridge->getMotorVelocities(m_jointSensorBuffer))
        {
            m_bufferManager.push_back(m_jointSensorBuffer, time, "motors_state::velocities");
        }
        if (m_robotSensorBridge->getMotorAccelerations(m_jointSensorBuffer))
        {
            m_bufferManager.push_back(m_jointSensorBuffer, time, "motors_state::accelerations");
        }
        if (m_robotSensorBridge->getMotorCurrents(m_jointSensorBuffer))
        {
            m_bufferManager.push_back(m_jointSensorBuffer, time, "motors_state::currents");
        }
    }

    if (m_streamMotorPWM)
    {
        if (m_robotSensorBridge->getMotorPWMs(m_jointSensorBuffer))
        {
            m_bufferManager.push_back(m_jointSensorBuffer, time, "motors_state::PWM");
        }
    }

    if (m_streamPIDs)
    {
        if (m_robotSensorBridge->getPidPositions(m_jointSensorBuffer))
        {
            m_bufferManager.push_back(m_jointSensorBuffer, time, "PIDs");
        }
    }

    for (const auto& sensorName : m_robotSensorBridge->getSixAxisForceTorqueSensorsList())
    {
        if (m_robotSensorBridge->getSixAxisForceTorqueMeasurement(sensorName, m_ftBuffer))
        {
            m_bufferManager.push_back(m_ftBuffer, time, "FTs::" + sensorName);
        }
    }

    for (const auto& sensorname : m_robotSensorBridge->getTemperatureSensorsList())
    {
        if (m_robotSensorBridge->getTemperature(sensorname, m_ftTemperatureBuffer))
        {
            m_bufferManager.push_back({m_ftTemperatureBuffer}, time, "temperatures::" + sensorname);
        }
    }

    for (const auto& sensorName : m_robotSensorBridge->getGyroscopesList())
    {
        if (m_robotSensorBridge->getGyroscopeMeasure(sensorName, m_gyroBuffer))
        {
            m_bufferManager.push_back(m_gyroBuffer, time, "gyros::" + sensorName);
        }
    }

    for (const auto& sensorName : m_robotSensorBridge->getLinearAccelerometersList())
    {
        if (m_robotSensorBridge->getLinearAccelerometerMeasurement(sensorName,
                                                                   m_acceloremeterBuffer))
        {
            m_bufferManager.push_back(m_acceloremeterBuffer, time, "accelerometers::" + sensorName);
        }
    }

    for (const auto& sensorName : m_robotSensorBridge->getOrientationSensorsList())
    {
        if (m_robotSensorBridge->getOrientationSensorMeasurement(sensorName, m_orientationBuffer))
        {
            m_bufferManager.push_back(m_orientationBuffer, time, "orientations::" + sensorName);
        }
    }

    // an IMU contains a gyro accelerometer and an orientation sensor
    for (const auto& sensorName : m_robotSensorBridge->getIMUsList())
    {
        if (m_robotSensorBridge->getIMUMeasurement(sensorName, m_analogSensorBuffer))
        {
            // it will return a tuple containing the Accelerometer, the gyro and the orientatio
            this->unpackIMU(m_analogSensorBuffer,
                            m_acceloremeterBuffer,
                            m_gyroBuffer,
                            m_orientationBuffer);

            m_bufferManager.push_back(m_acceloremeterBuffer, time, "accelerometers::" + sensorName);
            m_bufferManager.push_back(m_gyroBuffer, time, "gyros::" + sensorName);
            m_bufferManager.push_back(m_orientationBuffer, time, "orientations::" + sensorName);
        }
    }

    for (const auto& sensorName : m_robotSensorBridge->getCartesianWrenchesList())
    {
        if (m_robotSensorBridge->getCartesianWrench(sensorName, m_ftBuffer))
        {
            m_bufferManager.push_back(m_ftBuffer, time, "cartesian_wrenches::" + sensorName);
        }
    }

    std::string signalFullName;
    for (auto& [name, port] : m_exogenousPorts)
    {
        BipedalLocomotion::YarpUtilities::VectorsCollection* collection = port.read(false);
        if (collection != nullptr)
        {
            for (const auto& [key, vector] : collection->vectors)
            {
                signalFullName = name + "::" + key;

                // if it is the first time this signal is seen by the device the channel is added
                if (m_exogenousPortsStoredInManager.find(signalFullName)
                    == m_exogenousPortsStoredInManager.end())
                {
                    m_bufferManager.addChannel({signalFullName, {vector.size(), 1}});
                    m_exogenousPortsStoredInManager.insert(signalFullName);
                }

                m_bufferManager.push_back(vector, time, signalFullName);
            }
        }
    }

    int bufferportSize = m_textLoggingPort.getPendingReads();
    BipedalLocomotion::TextLoggingEntry msg;

    while (bufferportSize > 0)
    {
        yarp::os::Bottle* b = m_textLoggingPort.read(false);
        if (b != nullptr)
        {
            msg = BipedalLocomotion::TextLoggingEntry::deserializeMessage(*b, std::to_string(time));
            if (msg.isValid)
            {
                signalFullName = msg.portSystem + "::" + msg.portPrefix + "::" + msg.processName
                                 + "::p" + msg.processPID;

                // if it is the first time this signal is seen by the device the channel is added
                if (m_textLogsStoredInManager.find(signalFullName)
                    == m_textLogsStoredInManager.end())
                {
                    m_bufferManager.addChannel({signalFullName, {1, 1}});
                    m_textLogsStoredInManager.insert(signalFullName);
                }

                m_bufferManager.push_back(msg, time, signalFullName);
            }
            bufferportSize = m_textLoggingPort.getPendingReads();
        } else
        {
            break;
        }
    }
}

bool YarpRobotLoggerDevice::saveVideo(
    const std::string& fileName,
    const yarp::telemetry::experimental::SaveCallbackSaveMethod& method)
{
    for (const auto& camera : m_rgbCamerasList)
    {
        const std::string temp = fileName + "_" + camera + ".mp4";
        const std::string oldName = "output_" + camera + ".mp4";

        std::lock_guard<std::mutex> lock(m_videoWriters[camera].mutex);
        m_videoWriters[camera].writer->release();

        // rename the file associated to the camera
        std::rename(oldName.c_str(), temp.c_str());

        if (method == yarp::telemetry::experimental::SaveCallbackSaveMethod::periodic)
        {
            const auto& cameraInfo
                = m_cameraBridge->getMetaData().bridgeOptions.rgbImgDimensions.find(camera);
            m_videoWriters[camera].writer
                = std::make_shared<cv::VideoWriter>("output_" + camera + ".mp4",
                                                    cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                                                    m_videoWriters[camera].fps,
                                                    cv::Size(cameraInfo->second.first,
                                                             cameraInfo->second.second));
        }
    }

    return true;
}

bool YarpRobotLoggerDevice::detachAll()
{
    if (isRunning())
    {
        stop();
    }

    return true;
}

bool YarpRobotLoggerDevice::close()
{
    // stop all the video thread
    for (auto& [cameraName, writer] : m_videoWriters)
    {
        writer.recordVideoIsRunning = false;
    }

    // close all the thread associated to the video logging
    for (auto& [cameraName, writer] : m_videoWriters)
    {
        if (writer.videoThread.joinable())
        {
            writer.videoThread.join();
            writer.videoThread = std::thread();
        }
    }

    // close the thread associated to the text logging polling
    m_lookForNewLogsIsRunning = false;
    if (m_lookForNewLogsThread.joinable())
    {
        m_lookForNewLogsThread.join();
        m_lookForNewLogsThread = std::thread();
    }

    return true;
}
