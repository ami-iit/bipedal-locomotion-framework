/**
 * @file StdClock.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <chrono>
#include <thread>

#include <BipedalLocomotion/System/StdClock.h>

using namespace BipedalLocomotion::System;

std::chrono::duration<double> StdClock::now()
{
    return std::chrono::system_clock::now().time_since_epoch();
}

void StdClock::sleepFor(const std::chrono::duration<double>& sleepDuration)
{
    std::this_thread::sleep_for(sleepDuration);
}

void StdClock::sleepUntil(const std::chrono::duration<double>& time)
{
    std::this_thread::sleep_for(time - std::chrono::system_clock::now().time_since_epoch());
}

void StdClock::yield()
{
    std::this_thread::yield();
}

IClock& StdClockFactory::createClock()
{
    // Create the singleton. Meyers' implementation. It is automatically threadsafe
    static StdClock clock;
    return clock;
}
