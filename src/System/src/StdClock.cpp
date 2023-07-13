/**
 * @file StdClock.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <thread>

#include <BipedalLocomotion/System/StdClock.h>

using namespace BipedalLocomotion::System;

std::chrono::nanoseconds StdClock::now()
{
    return std::chrono::system_clock::now().time_since_epoch();
}

void StdClock::sleepFor(const std::chrono::nanoseconds& sleepDuration)
{
    std::this_thread::sleep_for(sleepDuration);
}

void StdClock::sleepUntil(const std::chrono::nanoseconds& time)
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
