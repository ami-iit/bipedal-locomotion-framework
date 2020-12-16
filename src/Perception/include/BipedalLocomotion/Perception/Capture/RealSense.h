/**
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

/**
 * Realsense driver class
 *
 * The following parameters are required to initialize the class
 *
 * |      Parameter Name     |       Type        |             Description                       | Mandatory | Default Value |
 * |:-----------------------:|:-----------------:|:---------------------------------------------:|:---------:|:-------------:|
 * |      `camera_name`      |      `string`     |         name of the camera device             |     No    |   RealSense   |
 * |      `frame_width`      |     `size_t`      |            image frame width                  |     No    |     640       |
*  |     `frame_height`      |     `size_t`      |            image frame height                 |     No    |     480       |
*  |          `fps`          |     `boolean`     |            frames per second                  |     No    |      30       |
*  |      `stream_color`     |     `boolean`     |      flag to enable streaming BGR images      |     No    |     false     |
*  |      `stream_depth`     |     `boolean`     |     flag to enable streaming depth images     |     No    |     false     |
*  |       `stream_ir`       |     `boolean`     |      flag to enable streaming IR images       |     No    |     false     |
*  |       `stream_pcl`      |     `boolean`     |      flag to enable streaming pointcloud      |     No    |     false     |
*  | `align_frames_to_color` |     `boolean`     |    flag to align other images to BGR images   |     No    |     false     |
*/

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

