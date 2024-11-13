/**
 * @file PassThroughControlBoardDevice.cpp
 * @authors Ines Sorrentino
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/PassThroughControlBoardDevice.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/YarpClock.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace BipedalLocomotion;

// CONSTRUCTOR/DESTRUCTOR
PassThroughControlBoardDevice::PassThroughControlBoardDevice(double period,
                                                   yarp::os::ShouldUseSystemClock useSystemClock)
    : PassThroughControlBoard(), yarp::os::PeriodicThread(period, useSystemClock)
{
    // Use the yarp clock in blf
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());
}

PassThroughControlBoardDevice::PassThroughControlBoardDevice()
    : PassThroughControlBoard(), yarp::os::PeriodicThread(0.002, yarp::os::ShouldUseSystemClock::No)
{
    // Use the yarp clock in blf
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());
}

PassThroughControlBoardDevice::~PassThroughControlBoardDevice()
{
}

// HIJACKED CONTROL THREAD
bool PassThroughControlBoardDevice::threadInit()
{
    return true;
}

void PassThroughControlBoardDevice::threadRelease()
{
    return;
}

// DEVICE DRIVER
bool PassThroughControlBoardDevice::open(yarp::os::Searchable& config)
{
    constexpr auto logPrefix = "[JointTorqueControlDevice::open]";

    // Call open of PassThroughControlBoard
    bool ret = PassThroughControlBoard::open(config);

    if (!ret)
    {
        return false;
    }

    PropertyConfigOptions.fromString(config.toString().c_str());

    this->setPeriod(0.002);

    return true;
}

bool PassThroughControlBoardDevice::attachAll(const PolyDriverList& p)
{
    return PassThroughControlBoard::attachAll(p);
}

bool PassThroughControlBoardDevice::detachAll()
{
    this->PeriodicThread::stop();
    return PassThroughControlBoard::detachAll();
}

bool PassThroughControlBoardDevice::close()
{
    return true;
}

void PassThroughControlBoardDevice::run()
{
    return;
}