/**
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H

#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <atomic>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>

#include <robometry/BufferManager.h>

#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/RobotInterface/YarpCameraBridge.h>

#include <BipedalLocomotion/YarpUtilities/VectorsCollection.h>

namespace BipedalLocomotion
{

class YarpRobotLoggerDevice : public yarp::dev::DeviceDriver,
                              public yarp::dev::IMultipleWrapper,
                              public yarp::os::PeriodicThread
{
public:
    YarpRobotLoggerDevice(double period,
                          yarp::os::ShouldUseSystemClock useSystemClock
                          = yarp::os::ShouldUseSystemClock::No);
    YarpRobotLoggerDevice();
    ~YarpRobotLoggerDevice();

    virtual bool open(yarp::os::Searchable& config) final;
    virtual bool close() final;
    virtual bool attachAll(const yarp::dev::PolyDriverList& poly) final;
    virtual bool detachAll() final;
    virtual void run() final;

private:
    using ft_t = Eigen::Matrix<double, 6, 1>;
    using gyro_t = Eigen::Matrix<double, 3, 1>;
    using accelerometer_t = Eigen::Matrix<double, 3, 1>;
    using orientation_t = Eigen::Matrix<double, 3, 1>;
    using magnemetometer_t = Eigen::Matrix<double, 3, 1>;
    using analog_sensor_t = Eigen::Matrix<double, 12, 1>;

    std::unique_ptr<BipedalLocomotion::RobotInterface::YarpSensorBridge> m_robotSensorBridge;
    std::unique_ptr<BipedalLocomotion::RobotInterface::YarpCameraBridge> m_cameraBridge;

    std::unordered_map<std::string,
                       yarp::os::BufferedPort<BipedalLocomotion::YarpUtilities::VectorsCollection>>
        m_exogenousPorts;
    std::unordered_set<std::string> m_exogenousPortsStoredInManager;

    std::vector<std::string> m_rgbCamerasList;
    struct VideoWriter{
        std::mutex mutex;
        std::shared_ptr<cv::VideoWriter> writer;
        cv::Mat frame;
        std::thread videoThread;
        std::atomic<bool> recordVideoIsRunning{false};
        int fps{-1};
    };

    std::unordered_map<std::string, VideoWriter> m_videoWriters;

    const std::string m_textLoggingPortName = "/YarpRobotLoggerDevice/TextLogging:i";
    std::unordered_set<std::string> m_textLoggingPortNames;
    yarp::os::BufferedPort<yarp::os::Bottle> m_textLoggingPort;
    std::atomic<bool> m_lookForNewLogsIsRunning{false};
    std::unordered_set<std::string> m_textLogsStoredInManager;
    std::thread m_lookForNewLogsThread;

    Eigen::VectorXd m_jointSensorBuffer;
    ft_t m_ftBuffer;
    gyro_t m_gyroBuffer;
    accelerometer_t m_acceloremeterBuffer;
    orientation_t m_orientationBuffer;
    magnemetometer_t m_magnemetometerBuffer;
    analog_sensor_t m_analogSensorBuffer;
    double m_ftTemperatureBuffer;

    bool m_streamMotorStates{false};
    bool m_streamJointStates{false};
    bool m_streamMotorPWM{false};
    bool m_streamPIDs{false};
    bool m_streamInertials{false};
    bool m_streamCartesianWrenches{false};
    bool m_streamFTSensors{false};
    bool m_streamTemperatureSensors{false};

    robometry::BufferManager m_bufferManager;

    void lookForNewLogs();
    void recordVideo(const std::string& cameraName, VideoWriter& writer);
    void unpackIMU(Eigen::Ref<const analog_sensor_t> signal,
                   Eigen::Ref<accelerometer_t> accelerometer,
                   Eigen::Ref<gyro_t> gyro,
                   Eigen::Ref<orientation_t> orientation);
    bool setupRobotSensorBridge(std::weak_ptr<const ParametersHandler::IParametersHandler> params);
    bool setupRobotCameraBridge(std::weak_ptr<const ParametersHandler::IParametersHandler> params);
    bool setupTelemetry(std::weak_ptr<const ParametersHandler::IParametersHandler> params,
                        const double& devicePeriod);
    bool setupExogenousInputs(std::weak_ptr<const ParametersHandler::IParametersHandler> params);

    bool saveVideo(const std::string& fileName,
                   const robometry::SaveCallbackSaveMethod& method);
};

} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H
