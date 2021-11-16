/**
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H

#include <matioCpp/matioCpp.h>

#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/PeriodicThread.h>

#include <memory>
#include <string>
#include <unordered_map>

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
    using analog_sensor_t = Eigen::Matrix<double, 12, 1>;

    std::unique_ptr<BipedalLocomotion::RobotInterface::YarpSensorBridge> m_robotSensorBridge;
    std::mutex m_deviceMutex;

    std::unordered_map<std::string, Eigen::MatrixXd> m_orientations;
    std::unordered_map<std::string, Eigen::MatrixXd> m_accelerometers;
    std::unordered_map<std::string, Eigen::MatrixXd> m_gyros;
    std::unordered_map<std::string, Eigen::MatrixXd> m_fts;
    std::unordered_map<std::string, Eigen::MatrixXd> m_jointState;
    std::unordered_map<std::string, Eigen::MatrixXd> m_motorState;
    std::unordered_map<std::string, Eigen::MatrixXd> m_motorPWMs;
    std::unordered_map<std::string, Eigen::MatrixXd> m_PIDs;
    Eigen::VectorXd m_time;
    double m_timeNow;

    std::vector<std::string> m_IMUNames;
    std::vector<std::string> m_FTNames;
    std::vector<std::string> m_accelerometerNames;
    std::vector<std::string> m_gyroNames;
    std::vector<std::string> m_orientationNames;
    std::vector<std::string> m_jointNames;
    analog_sensor_t m_analogSensorBuffer;
    unsigned int m_dofs;
    bool m_streamMotorStates{false};
    bool m_streamJointStates{false};
    bool m_streamMotorPWM{false};
    bool m_streamPIDs{false};

    void unpackIMU(Eigen::Ref<const analog_sensor_t> signal,
                   Eigen::Ref<accelerometer_t> accelerometer,
                   Eigen::Ref<gyro_t> gyro,
                   Eigen::Ref<orientation_t> orientation);

    matioCpp::Struct createStruct(const std::string& key,
                                  const std::unordered_map<std::string, Eigen::MatrixXd>& signal);

    bool setupRobotSensorBridge(yarp::os::Searchable& config);
    bool logData();
};

} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H
