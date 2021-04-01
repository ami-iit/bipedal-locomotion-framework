/**
 * @file YarpClock.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <yarp/os/Time.h>

#include <BipedalLocomotion/System/YarpClock.h>

using namespace BipedalLocomotion::System;

std::chrono::duration<double> YarpClock::now()
{
    // The yarp now function  returns the time in seconds
    return std::chrono::duration<double>(yarp::os::Time::now());
}

IClock& YarpClockFactory::createClock()
{
    // Create the singleton. Meyers' implementation. It is automatically threadsafe
    static YarpClock clock;
    return clock;
}
