/**
 * @file ICameraBridge.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_ICAMERA_BRIDGE_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_ICAMERA_BRIDGE_H

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>


namespace BipedalLocomotion
{
namespace RobotInterface
{

/**
 * Camera bridge options
 */
struct CameraBridgeOptions
{
    bool isRGBCameraEnabled{false}; /**< flag to connect RGB camera sources */
    bool isRGBDCameraEnabled{false}; /**< flag to connect RGBD camera sources */

    std::unordered_map<std::string, std::pair<std::size_t, std::size_t> > rgbImgDimensions; /**< dimensions of the images available through rgb camera streams, to be configured at initialization */
    std::unordered_map<std::string, std::pair<std::size_t, std::size_t> > rgbdImgDimensions; /**< dimensions of the depth images available through rgbd camera streams, to be configured at initialization */
};

/**
 *  Camera lists
 */
struct CameraLists
{
    std::vector<std::string> rgbCamerasList; /**< list of rgb cameras attached to the bridge */
    std::vector<std::string> rgbdCamerasList; /**< list of RGBD cameras attached to the bridge */
};


/**
 * Meta data struct to hold list of sensors and configured options
 * available from the Sensor bridge interface
 */
struct CameraBridgeMetaData
{
  CameraLists sensorsList;
  CameraBridgeOptions bridgeOptions;
};

/**
 * Sensor bridge interface.
 */
class ICameraBridge
{
public:
    using unique_ptr = std::unique_ptr<ICameraBridge>;

    using shared_ptr = std::shared_ptr<ICameraBridge>;

    using weak_ptr = std::weak_ptr<ICameraBridge>;

    /**
     * Initialize estimator
     * @param[in] handler Parameters handler
     */
    virtual bool initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler) = 0;   

    /**
     * Get rgb cameras
     * @param[out] rgbCamerasList list of rgb cameras attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getRGBCamerasList(std::vector<std::string>& rgbCamerasList) { return false; };

    /**
     * Get RGBD cameras
     * @param[out] rgbdCamerasList list of rgbd cameras attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getRGBDCamerasList(std::vector<std::string>& rgbdCamerasList) { return false; };

    /**
     * Get color image from the camera
     * @param[in] camName name of the camera
     * @param[out] colorImg image as cv Mat object
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     *
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "colorImg" to this method
     *
     * @return true/false in case of success/failure
     */
    virtual bool getColorImage(const std::string& camName,
                               cv::Mat& colorImg,
                               double* receiveTimeInSeconds = nullptr) { return false; };

    /**
     * Get depth image
     * @param[in] camName name of the gyroscope
     * @param[out] depthImg depth image as a cv Mat object
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     *
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "depthImg" to this method
     *
     * @return true/false in case of success/failure
     */
    virtual bool getDepthImage(const std::string& camName,
                               cv::Mat& depthImg,
                               double* receiveTimeInSeconds = nullptr) { return false; };

    /**
     * Destructor
     */
    virtual ~ICameraBridge() = default;

protected:
    /**
     * Helper method to maintain CameraBridgeOptions struct by populating it from the configuration parameters
     * @note the user may choose to use/not use this method depending on their requirements for the implementation
     * if the user chooses to not use the method, the implementation must simply contain "return true;"
     *
     * @param[in] handler  Parameters handler
     * @param[in] cameraBridgeOptions CameraBridgeOptions to hold the bridge options for streaming sensor measurements
     */
    virtual bool populateCameraBridgeOptionsFromConfig(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                                                      CameraBridgeOptions& cameraBridgeOptions) { return true; };

    /**
     * Helper method to maintain CameraLists struct by populating it from the configuration parameters
     * @note the user may choose to use/not use this method depending on their requirements for the implementation
     * if the user chooses to not use the method, the implementation must simply contain "return true;"
     *
     * @param[in] handler  Parameters handler
     * @param[in] cameraBridgeOptions configured object of CameraBridgeOptions
     * @param[in] cameraLists CameraLists object holding list of connected sensor devices
     */
    virtual bool populateCameraListsFromConfig(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                                               const CameraBridgeOptions& cameraBridgeOptions,
                                               CameraLists& cameraLists) { return true; };

    /**
     * Helper method to maintain CameraBridgeMetaData struct by populating it from the configuration parameters
     * @note the user may choose to use/not use this method depending on their requirements for the implementation
     * if the user chooses to not use the method, the implementation must simply contain "return true;"
     *
     * @param[in] handler  Parameters handler
     * @param[in] CameraBridgeMetaData configured object of CameraBridgeMetaData
     */
    virtual bool populateCameraBridgeMetaDataFromConfig(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                                                        CameraBridgeMetaData& cameraBridgeMetaData) { return true; };


};
} // namespace RobotInterface
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_ICAMERA_BRIDGE_H

