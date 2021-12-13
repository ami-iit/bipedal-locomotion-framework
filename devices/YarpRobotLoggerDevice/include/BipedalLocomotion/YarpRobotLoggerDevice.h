/**
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H

#include <yarp/telemetry/experimental/BufferManager.h>

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

    Eigen::VectorXd m_jointSensorBuffer;
    ft_t m_ftBuffer;
    gyro_t m_gyroBuffer;
    accelerometer_t m_acceloremeterBuffer;
    orientation_t m_orientationBuffer;
    analog_sensor_t m_analogSensorBuffer;

    bool m_streamMotorStates{false};
    bool m_streamJointStates{false};
    bool m_streamMotorPWM{false};
    bool m_streamPIDs{false};

    yarp::telemetry::experimental::BufferManager<double> m_bufferManager;

    void unpackIMU(Eigen::Ref<const analog_sensor_t> signal,
                   Eigen::Ref<accelerometer_t> accelerometer,
                   Eigen::Ref<gyro_t> gyro,
                   Eigen::Ref<orientation_t> orientation);
    bool setupRobotSensorBridge(std::weak_ptr<const ParametersHandler::IParametersHandler> params);
    bool setupTelemetry(std::weak_ptr<const ParametersHandler::IParametersHandler> params,
                        const double& devicePeriod);


};

} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H
