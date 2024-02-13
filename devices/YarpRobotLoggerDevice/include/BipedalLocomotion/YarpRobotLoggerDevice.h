/**
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/sig/Vector.h>

#include <robometry/BufferManager.h>

#include <BipedalLocomotion/RobotInterface/YarpCameraBridge.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollection.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionClient.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

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

    bool sendDataRT;
    BipedalLocomotion::YarpUtilities::VectorsCollectionServer m_vectorCollectionRTDataServer;

    template <typename T> struct ExogenousSignal
    {
        std::mutex mutex;
        std::string remote;
        std::string local;
        std::string carrier;
        std::string signalName;
        yarp::os::BufferedPort<T> port;
        bool dataArrived{false};
        bool connected{false};

        bool connect()
        {
            return yarp::os::Network::connect(remote, local, carrier);
        }

        void disconnect()
        {
            if (connected)
            {
                yarp::os::Network::disconnect(remote, local);
            }
        }
    };

    struct VectorsCollectionSignal
    {
        std::mutex mutex;
        BipedalLocomotion::YarpUtilities::VectorsCollectionClient client;
        BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata metadata;
        std::string signalName;
        bool dataArrived{false};
        bool connected{false};

        bool connect();
        void disconnect();
    };

    std::unordered_map<std::string, VectorsCollectionSignal> m_vectorsCollectionSignals;
    std::unordered_map<std::string, ExogenousSignal<yarp::sig::Vector>> m_vectorSignals;
    unsigned int m_updatedMetadataSignalVal;

    std::unordered_set<std::string> m_exogenousPortsStoredInManager;
    std::atomic<bool> m_lookForNewExogenousSignalIsRunning{false};
    std::thread m_lookForNewExogenousSignalThread;

    std::vector<std::string> m_rgbCamerasList;
    std::vector<std::string> m_rgbdCamerasList;
    struct VideoWriter
    {
        enum class SaveMode
        {
            Video,
            Frame
        };

        struct ImageSaver
        {
            std::mutex mutex;
            std::shared_ptr<cv::VideoWriter> writer;
            cv::Mat frame;
            SaveMode saveMode{SaveMode::Video};
            std::filesystem::path framesPath;
        };

        std::shared_ptr<ImageSaver> rgb;
        std::shared_ptr<ImageSaver> depth;
        int depthScale{1};

        std::thread videoThread;
        std::atomic<bool> recordVideoIsRunning{false};
        int fps{-1};
    };

    std::string m_videoCodecCode{"mp4v"};
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
    std::vector<std::string> m_textLoggingSubnames;
    std::vector<std::string> m_codeStatusCmdPrefixes;

    std::mutex m_bufferManagerMutex;
    std::mutex m_textLoggingPortMutex;
    robometry::BufferManager m_bufferManager;

    void lookForNewLogs();
    void lookForExogenousSignals();

    bool
    addChannelAndMetadata(const std::string& nameKey, const std::vector<std::string>& metadata);
    void storeAndSendLoggingData(const std::string& name,
                                 const Eigen::VectorXd& data,
                                 const double time);
    bool addChannel(const std::string& nameKey, const std::vector<std::string>& metadata);
    void storeLoggingData(const std::string& name,
                                 const Eigen::VectorXd& data,
                                 const double time);
    bool (BipedalLocomotion::YarpRobotLoggerDevice::*initMetadataFunction)(const std::string& nameKey, const std::vector<std::string>& metadata);
    void (BipedalLocomotion::YarpRobotLoggerDevice::*loggingDataFunction)(const std::string& name,
                                 const Eigen::VectorXd& data,
                                 const double time);

    bool hasSubstring(const std::string& str, const std::vector<std::string>& substrings) const;
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
    bool saveCallback(const std::string& fileName, const robometry::SaveCallbackSaveMethod& method);
    bool openVideoWriter(
        std::shared_ptr<VideoWriter::ImageSaver> imageSaver,
        const std::string& camera,
        const std::string& imageType,
        const std::unordered_map<std::string, std::pair<std::size_t, std::size_t>>& imgDimensions);
    bool createFramesFolder(std::shared_ptr<VideoWriter::ImageSaver> imageSaver,
                            const std::string& camera,
                            const std::string& imageType);

    const std::string TREE_DELIM = "::";

    const std::string ROBOT_RT_ROOT_NAME = "robot_realtime";

    const std::string JOINT_STATE_POSITIONS_NAME = "joints_state::positions";
    const std::string JOINT_STATE_VELOCITIES_NAME = "joints_state::velocities";
    const std::string JOINT_STATE_ACCLERATIONS_NAME = "joints_state::accelerations";
    const std::string JOINT_STATE_TORQUES_NAME = "joints_state::torques";

    const std::string MOTOR_STATE_POSITIONS_NAME = "motors_state::positions";
    const std::string MOTOR_STATE_VELOCITIES_NAME = "motors_state::velocities";
    const std::string MOTOR_STATE_ACCELERATIONS_NAME = "motors_state::acclerations";
    const std::string MOTOR_STATE_CURRENTS_NAME = "motors_state::currents";
    const std::string MOTOR_STATE_PWM_NAME = "motors_state::PWM";

    const std::string MOTOR_STATE_PIDS_NAME = "PIDs";

    const std::string FTS_NAME = "FTs";

    const std::vector<std::string> FTElementNames = {"f_x", "f_y", "f_z", "mu_x", "mu_y", "mu_z"};

    const std::string GYROS_NAME = "gyros";
    const std::vector<std::string> GyroElementNames = {"omega_x", "omega_y", "omega_z"};

    const std::string ACCELEROMETERS_NAME = "accelerometers";
    const std::vector<std::string> AccelerometerElementNames = {"a_x", "a_y", "a_z"};

    const std::string ORIENTATIONS_NAME = "orientations";
    const std::vector<std::string> OrientationElementNames = {"r", "p", "y"};

    const std::string MAGNETOMETERS_NAME = "magnetometers";
    const std::vector<std::string> MagnetometerElementNames = {"mag_x", "mag_y", "mag_z"};

    const std::string CARTESIAN_WRENCHES_NAME = "cartesian_wrenches";
    const std::vector<std::string> CartesianWrenchNames = {FTElementNames[0],
                                                        FTElementNames[1],
                                                        FTElementNames[2],
                                                        FTElementNames[3],
                                                        FTElementNames[4],
                                                        FTElementNames[5]};

    const std::string TEMPERATURE_NAME = "temperatures";
    const std::vector<std::string> TemperatureNames = {"temperature"};

    const std::string YARP_NAME = "yarp_robot_name";

    const std::string ROBOT_DESCRIPTON_LIST = "description_list";

    const std::string TIMESTAMPS_NAME = "timestamps";
};

} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H
