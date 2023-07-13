/**
 * @file RosClock.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <thread>

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include <BipedalLocomotion/System/RosClock.h>

using namespace BipedalLocomotion::System;

std::chrono::nanoseconds RosClock::now()
{
    return std::chrono::nanoseconds(m_clock.now().nanoseconds());
}

void RosClock::sleepFor(const std::chrono::nanoseconds& sleepDuration)
{
    // std::chrono::duration store the time in second
    m_clock.sleep_for(sleepDuration);
}

void RosClock::sleepUntil(const std::chrono::nanoseconds& sleepTime)
{
    m_clock.sleep_for(sleepTime - this->now());
}

void RosClock::yield()
{
}

RosClockFactory::RosClockFactory(int argc, char const* argv[])
    : ClockFactory()
{
    try
    {
        rclcpp::init(argc, argv);
    } catch (const rclcpp::ContextAlreadyInitialized& except)
    {
    };
}

RosClockFactory::RosClockFactory(const std::vector<std::string>& args)
    : ClockFactory()
{
    std::vector<const char*> rargs(args.size(), 0);
    for (int i = 0; i < args.size(); ++i)
    {
        rargs[i] = args[i].c_str();
    }
    try
    {
        rclcpp::init(rargs.size(), rargs.data());
    } catch (const rclcpp::ContextAlreadyInitialized& except)
    {
    };
}

IClock& RosClockFactory::createClock()
{
    // Create the singleton. Meyers' implementation. It is automatically threadsafe
    static RosClock clock;
    return clock;
}
