/**
 * @file StdClock.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <thread>

#if defined(_WIN32)
#include <Windows.h>
#endif

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

StdClock::PrecisionScheduler::PrecisionScheduler()
{
#if defined(_WIN32)
    // Only affects Windows systems.
    TIMECAPS tm; // Stores system timer capabilities.
    // Get the minimum timer resolution supported by the system.
    timeGetDevCaps(&tm, sizeof(TIMECAPS));
    // Set the system timer resolution to the minimum value for higher precision.
    timeBeginPeriod(tm.wPeriodMin);
#endif
}

StdClock::PrecisionScheduler::~PrecisionScheduler()
{
#if defined(_WIN32)
    // Only affects Windows systems.
    TIMECAPS tm; // Stores system timer capabilities.
    // Get the minimum timer resolution supported by the system.
    timeGetDevCaps(&tm, sizeof(TIMECAPS));
    // Restore the system timer resolution to the default value.
    timeEndPeriod(tm.wPeriodMin);
#endif
}

IClock& StdClockFactory::createClock()
{
    // Create the singleton. Meyers' implementation. It is automatically threadsafe
    static StdClock clock;
    return clock;
}
