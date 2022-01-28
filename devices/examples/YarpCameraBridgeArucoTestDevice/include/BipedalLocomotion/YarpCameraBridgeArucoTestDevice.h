/**
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_CAMERA_BRIDGE_ARUCO_TEST_DEVICE_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_CAMERA_BRIDGE_ARUCO_TEST_DEVICE_H

#include <memory>
#include <string>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>

#include <BipedalLocomotion/RobotInterface/YarpCameraBridge.h>
#include <BipedalLocomotion/Perception/Features/ArucoDetector.h>

namespace BipedalLocomotion
{

class YarpCameraBridgeArucoTestDevice : public yarp::dev::DeviceDriver,
                                        public yarp::dev::IMultipleWrapper,
                                        public yarp::os::PeriodicThread
{
public:
    YarpCameraBridgeArucoTestDevice(double period,
                                    yarp::os::ShouldUseSystemClock useSystemClock
                                    = yarp::os::ShouldUseSystemClock::No);
    YarpCameraBridgeArucoTestDevice();
    ~YarpCameraBridgeArucoTestDevice();

    virtual bool open(yarp::os::Searchable& config) final;
    virtual bool close() final;
    virtual bool attachAll(const yarp::dev::PolyDriverList& poly) final;
    virtual bool detachAll() final;
    virtual void run() final;

private:
    bool setupRobotCameraBridge(std::weak_ptr<const ParametersHandler::IParametersHandler> params);
    bool setupArucoDetector(std::weak_ptr<const ParametersHandler::IParametersHandler> params);
    std::unique_ptr<BipedalLocomotion::RobotInterface::YarpCameraBridge> m_robotCameraBridge;
    BipedalLocomotion::Perception::ArucoDetector m_arucoDetector;
    cv::Mat m_rgbImg, m_depthImg;
};

} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_CAMERA_BRIDGE_ARUCO_TEST_DEVICE_H
