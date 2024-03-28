/**
 * @file TimeProfiler.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <ctime>
#include <iostream>
#include <string>

#include <BipedalLocomotion/TimeProfiler/TimeProfiler.h>

using namespace BipedalLocomotion;

const double& Timer::getAverageDuration() const
{
    return m_averageDuration;
}

void Timer::resetAverageDuration()
{
    m_averageDuration = 0;
}

void Timer::setInitTime()
{
    m_initTime = clock();
}

void Timer::setEndTime()
{
    m_endTime = clock();
}

void Timer::evaluateDuration()
{
    clock_t duration = m_endTime - m_initTime;
    m_averageDuration += static_cast<double>(duration) / CLOCKS_PER_SEC * 1000.0;
}

void TimeProfiler::setPeriod(int maxCounter)
{
    m_maxCounter = maxCounter;
}

bool TimeProfiler::addTimer(const std::string& key)
{
    auto timer = m_timers.find(key);
    if(timer != m_timers.end())
    {
        std::cerr << "[TimeProfiler::addTimer] This timer already exist." <<std::endl;
        return false;
    }

    m_timers.insert(std::make_pair(key, std::make_unique<Timer>()));
    return true;
}

bool TimeProfiler::setInitTime(const std::string& key)
{
    auto timer = m_timers.find(key);
    if(timer == m_timers.end())
    {
        std::cerr << "[TimeProfiler::setInitTime] Unable to find the timer." <<std::endl;
        return false;
    }

    timer->second->setInitTime();
    return true;
}

bool TimeProfiler::setEndTime(const std::string& key)
{
    auto timer = m_timers.find(key);
    if(timer == m_timers.end())
    {
        std::cerr << "[TimeProfiler::setEndTime] Unable to find the timer." <<std::endl;
        return false;
    }

    timer->second->setEndTime();
    return true;
}

void TimeProfiler::profiling()
{
    std::string infoStream;
    m_counter++;
    for(auto timer = m_timers.begin(); timer != m_timers.end(); timer++)
    {
        timer->second->evaluateDuration();
        if(m_counter == m_maxCounter)
        {
            infoStream += timer->first + ": "
                + std::to_string((timer->second->getAverageDuration())/m_counter)
                + " ms ";
            timer->second->resetAverageDuration();
        }
    }
    if(m_counter == m_maxCounter)
    {
        m_counter = 0;
        std::cout << infoStream << std::endl;
    }
}
