/**
 * @file ROSPublisherTestDevice.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/ROSPublisherTestDevice.h>
#include <BipedalLocomotion/YarpUtilities/Helper.h>
#include <BipedalLocomotion/GenericContainer/Vector.h>
#include <yarp/os/LogStream.h>

BipedalLocomotion::ROSPublisherTestDevice::ROSPublisherTestDevice(double period,
                                               yarp::os::ShouldUseSystemClock useSystemClock)
        : yarp::os::PeriodicThread(period, useSystemClock)
{
	pub = std::make_unique<BipedalLocomotion::YarpUtilities::RosPublisher>("/PublisherTest");
}

BipedalLocomotion::ROSPublisherTestDevice::ROSPublisherTestDevice()
        : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No)
{
	pub = std::make_unique<BipedalLocomotion::YarpUtilities::RosPublisher>("/PublisherTest");
}

BipedalLocomotion::ROSPublisherTestDevice::~ROSPublisherTestDevice()
{
}

bool BipedalLocomotion::ROSPublisherTestDevice::open(yarp::os::Searchable& config)
{
    std::shared_ptr<BipedalLocomotion::ParametersHandler::YarpImplementation> configHandler = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    configHandler->set(config);
	
    if (!pub->initialize(configHandler))
    {
    	return false;
    }
	
    this->start();
    return true;
}

void BipedalLocomotion::ROSPublisherTestDevice::run()
{
   std::vector<std::string> jointList{"hello"};
   std::vector<double> jointPos{20.0};
   
   pub->publishJointStates(jointList, jointPos);
   
   std::vector<double> wrench{0.0, 1.0, 2.0,0.0, 0.0, 0.0};
   pub->publishWrench("right", wrench);
   
   
   std::vector<double> pose{1., 0, 0, 0, 0, 1., 0, 0, 0, 0, 1., 0, 0, 0, 0, 1.};
   pub->publishTransform("/world", "/dummy", pose);
}


bool BipedalLocomotion::ROSPublisherTestDevice::close()
{
    this->stop();
    pub->stop();
    return true;
}

