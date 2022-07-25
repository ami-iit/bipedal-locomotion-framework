/**
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_IPCL_BRIDGE_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_IPCL_BRIDGE_H

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace BipedalLocomotion
{
namespace RobotInterface
{

/**
 *  Device lists
 */
struct PCLDeviceLists
{
    std::vector<std::string> pclDevList; /**< list of PCL cameras attached to the bridge */
};

/**
 * Meta data struct to hold list of sensors and configured options
 * available from the Sensor bridge interface
 */
struct PCLBridgeMetaData
{
    PCLDeviceLists pclDevList;
};

/**
 * Sensor bridge interface.
 */
class IPointCloudBridge
{
public:
    using unique_ptr = std::unique_ptr<IPointCloudBridge>;

    using shared_ptr = std::shared_ptr<IPointCloudBridge>;

    using weak_ptr = std::weak_ptr<IPointCloudBridge>;

    /**
     * Initialize estimator
     * @param[in] handler Parameters handler
     */
    virtual bool
    initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
        = 0;

    /**
     * Get RGBD cameras
     * @param[out] rgbdCamerasList list of rgbd cameras attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getPCLDevicesList(std::vector<std::string>& pclDevList)
    {
        return false;
    };

    virtual bool
    getPointCloud(const std::string& pclDev,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredPointCloud,
                  std::optional<std::reference_wrapper<double>> receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Destructor
     */
    virtual ~IPointCloudBridge() = default;
};
} // namespace RobotInterface
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_IPCL_BRIDGE_H
