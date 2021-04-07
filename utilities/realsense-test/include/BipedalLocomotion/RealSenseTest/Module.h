/**
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_UTILITIES_REALSENSE_TEST_MODULE_H
#define BIPEDAL_LOCOMOTION_UTILITIES_REALSENSE_TEST_MODULE_H

// std
#include <memory>
#include <string>
#include <vector>

// YARP
#include <yarp/os/RFModule.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Perception/Capture/RealSense.h>
#include <BipedalLocomotion/Perception/Features/PointCloudProcessor.h>

#include <memory>

#include <pcl/visualization/cloud_viewer.h>

namespace BipedalLocomotion
{
namespace RealSenseTest
{

class Module : public yarp::os::RFModule
{
    double m_dT; /**< RFModule period. */
    std::string m_robot; /**< Robot name. */
    std::unique_ptr<BipedalLocomotion::Perception::Capture::RealSense> rsDev;
    std::string colorImgName{"Color Image"};
    std::string depthImgName{"Depth Image"};
    std::string irImgName{"IR Image"};

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcRaw{nullptr};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcProcessed{nullptr};
    pcl::shared_ptr< pcl::visualization::PCLVisualizer> viewer;
    pcl::shared_ptr< pcl::visualization::PCLVisualizer> viewerProcessed;
    BipedalLocomotion::Perception::Features::PointCloudProcessor<pcl::PointXYZRGB> pclProc;

    int previousClusterSize{0}; // used for visualization
    Eigen::Matrix4f w_H_cam;
    bool m_showImages{false};

public:
    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() override;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() override;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder& rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;
};
} // namespace RealSenseTest
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_UTILITIES_REALSENSE_TEST_MODULE_H
