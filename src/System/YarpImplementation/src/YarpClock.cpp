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

void YarpClock::sleepFor(const std::chrono::duration<double>& sleepDuration)
{
    // std::chrono::duration store the time in second
    yarp::os::Time::delay(sleepDuration.count());
}

void YarpClock::sleepUntil(const std::chrono::duration<double>& sleepTime)
{
    yarp::os::Time::delay(
        (sleepTime - std::chrono::duration<double>(yarp::os::Time::now())).count());
}

IClock& YarpClockFactory::createClock()
{
    // Create the singleton. Meyers' implementation. It is automatically threadsafe
    static YarpClock clock;
    return clock;
}
