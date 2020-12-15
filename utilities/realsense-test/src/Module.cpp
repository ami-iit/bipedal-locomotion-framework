/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <fstream>
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

    auto rsptr = parametersHandler->getGroup("REALSENSE").lock();
    if (rsptr == nullptr)
    {
        yError() << "[Module::configure] Robot interface options is empty.";
        return false;
    }
    
    rsDev = std::make_unique<Perception::Capture::RealSense>();
    if (!rsDev->initialize(rsptr))
    {
        yError() << "[Module::configure] could not initialize realsense camera.";
        return false;
    }
    
    std::cout << "[Module::configure] Starting the experiment." << std::endl;
        
    
    pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();    
    viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Realsense PCL");

    viewer->setBackgroundColor(0, 0, 0);
    viewer->resetCamera();

    viewer->spinOnce();
    return true;
}


bool Module::updateModule()
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
    
    if (rsDev->getPointCloud("D435i", pc))
    {
        viewer->removePointCloud("srcCloud");
        if (pc != nullptr)
        {
            viewer->addPointCloud(pc, "srcCloud");
            viewer->spinOnce();  
        }        
    }
    
    return true;
}

bool Module::close()
{
    return true;
}
