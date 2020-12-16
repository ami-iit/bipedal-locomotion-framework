/**
 * @file RealSense.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */


#ifndef BIPEDAL_LOCOMOTION_PERCEPTION_CAPTURE_REALSENSE_H
#define BIPEDAL_LOCOMOTION_PERCEPTION_CAPTURE_REALSENSE_H

#include <BipedalLocomotion/RobotInterface/ICameraBridge.h>
#include <BipedalLocomotion/RobotInterface/IPointCloudBridge.h>

namespace BipedalLocomotion {

namespace Perception {
    
namespace Capture {

    
class RealSense : public BipedalLocomotion::RobotInterface::ICameraBridge,
                  public BipedalLocomotion::RobotInterface::IPointCloudBridge
{
    public:
    RealSense();

    ~RealSense();

    bool initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler) final;
    
    bool isValid();

    bool getRGBCamerasList(std::vector<std::string>& rgbCamerasList) final;

    bool getRGBDCamerasList(std::vector<std::string>& rgbdCamerasList) final;

    bool getPCLDevicesList(std::vector<std::string>& pclDevList) final;

    bool getColorImage(const std::string& camName,
                       cv::Mat& colorImg,
                       double* receiveTimeInSeconds = nullptr) final;

    bool getDepthImage(const std::string& camName,
                       cv::Mat& depthImg,
                       double* receiveTimeInSeconds = nullptr) final;
                       
   bool getColorizedDepthImage(const std::string& camName,
                               cv::Mat& depthImg,
                               double* receiveTimeInSeconds = nullptr);
                       
    bool getInfraredImage(const std::string& camName,
                          cv::Mat& irImage,
                          double* receiveTimeInSeconds = nullptr);
                           
    bool getPointCloud(const std::string& pclDevName,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredPointCloud,
                       double* receiveTimeInSeconds = nullptr) final;

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
    
};


} // namespace Capture  
} // namespace Perception    
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PERCEPTION_CAPTURE_REALSENSE_H

