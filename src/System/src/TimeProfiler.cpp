/**
 * @file TimeProfiler.cpp
 * @authors Guglielmo Cervettini, Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <string>

#include <BipedalLocomotion/System/TimeProfiler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::System;

const std::chrono::nanoseconds& Timer::getAverageDuration() const
{
    return m_averageDuration;
}

void Timer::resetAverageDuration()
{
    m_averageDuration = std::chrono::nanoseconds(0);
}

void Timer::setInitTime()
{
    m_initTime = std::chrono::steady_clock::now();
}

void Timer::setEndTime()
{
    m_endTime = std::chrono::steady_clock::now();
}

void Timer::evaluateDuration()
{
    const std::chrono::nanoseconds duration = m_endTime - m_initTime;
    m_averageDuration += duration;
}

void TimeProfiler::setPeriod(int maxCounter)
{
    m_maxCounter = maxCounter;
}

bool TimeProfiler::addTimer(const std::string& key)
{
    auto timer = m_timers.find(key);
    if (timer != m_timers.end())
    {
        log()->error("[TimeProfiler::addTimer] The timer named {} already exists.", key);
        return false;
    }

    m_timers.insert(std::make_pair(key, Timer()));
    return true;
}

bool TimeProfiler::setInitTime(const std::string& key)
{
    auto timer = m_timers.find(key);
    if (timer == m_timers.end())
    {
        log()->error("[TimeProfiler::setInitTime] Unable to find the timer named {}.", key);
        return false;
    }

    timer->second.setInitTime();
    return true;
}

bool TimeProfiler::setEndTime(const std::string& key)
{
    auto timer = m_timers.find(key);
    if (timer == m_timers.end())
    {
        log()->error("[TimeProfiler::setEndTime] Unable to find the timer named {}.", key);
        return false;
    }

    timer->second.setEndTime();
    return true;
}

void TimeProfiler::profiling()
{
    std::string infoStream;
    m_counter++;
    for (auto& [key, timer] : m_timers)
    {
        timer.evaluateDuration();
        if (m_counter != m_maxCounter)
        {
            continue;
        }

        const std::chrono::nanoseconds& durationInNs = timer.getAverageDuration();

        // convert the duration in ns to ms
        const auto durationInMs
            = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(durationInNs)
              / double(m_counter);

        infoStream += key + ": " + std::to_string(durationInMs.count()) + " ms ";
        timer.resetAverageDuration();
    }

    if (m_counter == m_maxCounter)
    {
        m_counter = 0;
        log()->info("[TimeProfiler::profiling] {}", infoStream);
    }
}
