/**
 * @file ROSPublisherTestDevice.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_ROS_PUBLISHER_TEST_DEVICE_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_ROS_PUBLISHER_TEST_DEVICE_H

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/YarpUtilities/RosPublisher.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>

#include <memory>

namespace BipedalLocomotion
{
    class ROSPublisherTestDevice;
}

class BipedalLocomotion::ROSPublisherTestDevice : public yarp::dev::DeviceDriver,
                                                  public yarp::os::PeriodicThread
{
public:
    explicit ROSPublisherTestDevice(double period, yarp::os::ShouldUseSystemClock useSystemClock = yarp::os::ShouldUseSystemClock::No);
    ROSPublisherTestDevice();
    ~ROSPublisherTestDevice();

    virtual bool open(yarp::os::Searchable& config) final;
    virtual bool close() final;
    virtual void run() final;
    
    std::unique_ptr<BipedalLocomotion::YarpUtilities::RosPublisher> pub;
};



#endif //BIPEDAL_LOCOMOTION_FRAMEWORK_ROS_PUBLISHER_TEST_DEVICE_H
