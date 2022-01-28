/**
 * @copyright 2020, 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <memory>
#include <tuple>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/TextLogging/LoggerBuilder.h>
#include <BipedalLocomotion/TextLogging/YarpLogger.h>
#include <BipedalLocomotion/YarpCameraBridgeArucoTestDevice.h>

using namespace BipedalLocomotion::YarpUtilities;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion;

YarpCameraBridgeArucoTestDevice::YarpCameraBridgeArucoTestDevice(double period,
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

YarpCameraBridgeArucoTestDevice::YarpCameraBridgeArucoTestDevice()
    : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No)
{
    // Use the yarp clock in blf
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());

    // the logging message are streamed using yarp
    BipedalLocomotion::TextLogging::LoggerBuilder::setFactory(
        std::make_shared<BipedalLocomotion::TextLogging::YarpLoggerFactory>());
}

YarpCameraBridgeArucoTestDevice::~YarpCameraBridgeArucoTestDevice() = default;

bool YarpCameraBridgeArucoTestDevice::open(yarp::os::Searchable& config)
{
    auto params = std::make_shared<ParametersHandler::YarpImplementation>(config);

    double devicePeriod{0.01};
    if (params->getParameter("sampling_period_in_s", devicePeriod))
    {
        this->setPeriod(devicePeriod);
    }

    if (!this->setupRobotCameraBridge(params->getGroup("RobotCameraBridge")))
    {
        return false;
    }

    if (!this->setupArucoDetector(params->getGroup("ArucoDetector")))
    {
        return false;
    }

    return true;
}


bool YarpCameraBridgeArucoTestDevice::setupRobotCameraBridge(
    std::weak_ptr<const ParametersHandler::IParametersHandler> params)
{
    constexpr auto logPrefix = "[YarpCameraBridgeArucoTestDevice::setupRobotSensorBridge]";

    auto ptr = params.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameters handler is not valid.", logPrefix);
        return false;
    }

    m_robotCameraBridge = std::make_unique<YarpCameraBridge>();
    if (!m_robotCameraBridge->initialize(ptr))
    {
        log()->error("{} Unable to configure the 'YarpCameraBridge'", logPrefix);
        return false;
    }


    return true;
}

bool YarpCameraBridgeArucoTestDevice::setupArucoDetector(
    std::weak_ptr<const ParametersHandler::IParametersHandler> params)
{
    constexpr auto logPrefix = "[YarpCameraBridgeArucoTestDevice::setupArucoDetector]";

    auto ptr = params.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameters handler is not valid.", logPrefix);
        return false;
    }

    if (!m_arucoDetector.initialize(ptr))
    {
        log()->error("{} Unable to configure the 'ArucoDetector'", logPrefix);
        return false;
    }

    return true;
}

bool YarpCameraBridgeArucoTestDevice::attachAll(const yarp::dev::PolyDriverList& poly)
{
    constexpr auto logPrefix = "[YarpCameraBridgeArucoTestDevice::attachAll]";
    bool ok = true;
    if (!m_robotCameraBridge->setDriversList(poly))
    {
        log()->error("{} Could not attach drivers list to sensor bridge.", logPrefix);
        return false;
    }

    if (ok)
    {
        return start();
    }
    return ok;
}

void YarpCameraBridgeArucoTestDevice::run()
{
    constexpr auto logPrefix = "[YarpCameraBridgeArucoTestDevice::run]";

    const double time = BipedalLocomotion::clock().now().count();
    double imgRcvTime;
    bool ok = m_robotCameraBridge->getColorImage("rgbd_cam", m_rgbImg, imgRcvTime);
    if (ok)
    {
        cv::imshow("rgb", m_rgbImg);
        cv::waitKey(1);
    }

    ok = m_robotCameraBridge->getDepthImage("rgbd_cam", m_depthImg);
    if (ok)
    {
        cv::imshow("depth", m_depthImg);
        cv::waitKey(1);
    }

    m_arucoDetector.setImage(m_rgbImg, imgRcvTime);
    m_arucoDetector.advance();

    cv::Mat outputImage;
    m_arucoDetector.getImageWithDetectedMarkers(outputImage,
                                                /*drawFrames=*/ true,
                                                /*axisLengthForDrawing=*/ 0.3);
    if (!outputImage.empty())
    {
        cv::imshow("outputImage", outputImage);
        cv::waitKey(1);
    }
}

bool YarpCameraBridgeArucoTestDevice::detachAll()
{
    if (isRunning())
    {
        stop();
    }

    return true;
}

bool YarpCameraBridgeArucoTestDevice::close()
{
    return true;
}
