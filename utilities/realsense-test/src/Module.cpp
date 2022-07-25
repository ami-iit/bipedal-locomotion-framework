/**
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <fstream>
#include <string>
#include <iomanip>
#include <yarp/os/LogStream.h>
#include <BipedalLocomotion/RealSenseTest/Module.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::RealSenseTest;

double Module::getPeriod()
{
    return m_dT;
}

bool Module::configure(yarp::os::ResourceFinder& rf)
{
    auto parametersHandler = std::make_shared<ParametersHandler::YarpImplementation>(rf);

    if(!parametersHandler->getParameter("sampling_time", m_dT))
    {
        return false;
    }

    if(!parametersHandler->getParameter("show_images", m_showImages))
    {
        return false;
    }

    auto rsptr = parametersHandler->getGroup("REALSENSE").lock();
    if (rsptr == nullptr)
    {
        yError() << "[Module::configure] REALSENSE options is empty.";
        return false;
    }

    rsDev = std::make_unique<Perception::Capture::RealSense>();
    if (!rsDev->initialize(rsptr))
    {
        yError() << "[Module::configure] could not initialize realsense camera.";
        return false;
    }


    auto pclptr = parametersHandler->getGroup("PCL_PROCESSING").lock();
    if (pclptr == nullptr)
    {
        yError() << "[Module::configure] PCL_PROCESSING options is empty.";
        return false;
    }
    if (!pclProc.initialize(pclptr))
    {
        yError() << "[Module::configure] could not initialize PCL processor.";
        return false;
    }

    std::cout << "[Module::configure] Starting the experiment." << std::endl;


    pcRaw = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcProcessed = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    viewer = pcl::make_shared<pcl::visualization::PCLVisualizer>("Realsense PCL");
    viewerProcessed = pcl::make_shared<pcl::visualization::PCLVisualizer>("PCL Processed");

    viewer->setBackgroundColor(0, 0, 0);
    viewer->resetCamera();
    viewer->addCoordinateSystem();
    viewer->spinOnce();

    viewerProcessed->setBackgroundColor(0, 0, 0);
    viewerProcessed->resetCamera();
    viewerProcessed->addCoordinateSystem();
    viewerProcessed->spinOnce();

    w_H_cam << 0,  0, 1, 0,
              -1,  0, 0, 0,
               0, -1, 0, 0,
               0,  0, 0, 1;
    return true;
}


bool Module::updateModule()
{
    if (m_showImages)
    {
        cv::Mat bgr;
        if (rsDev->getColorImage("D435i", bgr))
        {
            cv::namedWindow(colorImgName);
            cv::imshow(colorImgName, bgr);
            cv::waitKey(1);
        }

        cv::Mat depth;
        if (rsDev->getColorizedDepthImage("D435i", depth))
        {
            cv::namedWindow(depthImgName);
            cv::imshow(depthImgName, depth);
            cv::waitKey(1);
        }


        cv::Mat ir;
        if (rsDev->getInfraredImage("D435i", ir))
        {
            cv::namedWindow(irImgName);
            cv::imshow(irImgName, ir);
            cv::waitKey(1);
        }
    }

    if (rsDev->getPointCloud("D435i", pcRaw))
    {
        viewer->removePointCloud("srcCloud");
        if (pcRaw != nullptr)
        {

            viewer->addPointCloud(pcRaw, "srcCloud");
            viewer->spinOnce();

            // first downsample then remove outliers
            pclProc.downsample(pcRaw, pcProcessed);
            pclProc.removeOutliers(pcProcessed, pcProcessed);

            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
            std::vector<pcl::PointIndices> clusterIndices;
            if (!pclProc.extractClusters(pcProcessed, clusterIndices, clusters))
            {
                yInfo() << "Extraction failure." ;
            }
            yInfo() << "Nr of clusters: " << clusters.size();
            for (size_t idx = 0; idx < previousClusterSize; idx++)
            {
                if (viewerProcessed->contains(std::to_string(idx)))
                {
                    viewerProcessed->removePointCloud(std::to_string(idx));
                }
            }

            for (size_t idx = 0; idx < clusters.size(); idx++)
            {
                auto pcTrans = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                pclProc.transform(clusters[idx], w_H_cam, pcTrans);
                viewerProcessed->addPointCloud(pcTrans, std::to_string(idx));
            }
            previousClusterSize = clusters.size();

            viewerProcessed->spinOnce();
        }
    }

    return true;
}

bool Module::close()
{
    return true;
}
