/**
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_FT_IMU_LOGGER_DEVICE_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_FT_IMU_LOGGER_DEVICE_H

#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Wrapper.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace BipedalLocomotion
{
    class FTIMULoggerDevice;
}

class BipedalLocomotion::FTIMULoggerDevice : public yarp::dev::DeviceDriver,
                                             public yarp::dev::IMultipleWrapper,
                                             public yarp::os::PeriodicThread
{
public:
    explicit FTIMULoggerDevice(double period, yarp::os::ShouldUseSystemClock useSystemClock = yarp::os::ShouldUseSystemClock::No);
    FTIMULoggerDevice();
    ~FTIMULoggerDevice();

    virtual bool open(yarp::os::Searchable& config) final;
    virtual bool close() final;
    virtual bool attachAll(const yarp::dev::PolyDriverList & poly) final;
    virtual bool detachAll() final;
    virtual void run() final;

    struct FTIMUPair
    {
        // todo: make it a circular buffer
        // with periodic flushing to the file
        Eigen::MatrixXd ft;
        Eigen::MatrixXd acc;
        Eigen::MatrixXd gyro;
        Eigen::MatrixXd orient;
    };

private:
    bool setupRobotSensorBridge(yarp::os::Searchable& config);
    bool logData();
    std::string m_robot{"iCub"};
    std::string m_portPrefix{"/ft_imu_logger"};
    std::unique_ptr<BipedalLocomotion::RobotInterface::YarpSensorBridge> m_robotSensorBridge;
    std::mutex m_deviceMutex;

    std::unordered_map<std::string, FTIMUPair> m_ftimupair;

    Eigen::VectorXd time;

    double timeNow;
    Eigen::Matrix<double, 3, 1> acc;
    Eigen::Matrix<double, 3, 1> gyro;
    Eigen::Matrix<double, 3, 1> orient;
    Eigen::Matrix<double, 6, 1> ft;
};



#endif //BIPEDAL_LOCOMOTION_FRAMEWORK_FT_IMU_LOGGER_DEVICE_H

