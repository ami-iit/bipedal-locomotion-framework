/**
 * @file RosClock.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_ROS_CLOCK_H
#define BIPEDAL_LOCOMOTION_SYSTEM_ROS_CLOCK_H

#include <BipedalLocomotion/System/IClock.h>

#include <chrono>

#include <rclcpp/clock.hpp>

namespace BipedalLocomotion
{
namespace System
{

/**
 * RosClock implements the IClock interface using ros functions.
 * The clock can be easily used as follows
 * \code{.cpp}
 * #include <BipedalLocomotion/System/Clock.h>
 * #include <BipedalLocomotion/System/RosClock.h>
 *
 * #include <chrono>
 * #include <memory>
 *
 * int main(int argc, char *argv[])
 * {
 *    // Change the clock
 *    BipedalLocomotion::System::ClockBuilder::setFactory(std::make_shared<BipedalLocomotion::System::RosClockFactory>(argc, argv));
 *
 *    // Add a sleep
 *    BipedalLocomotion::clock().sleepFor(2000ms);
 *
 *    auto start = BipedalLocomotion::clock().now();
 *    foo();
 *    auto end = BipedalLocomotion::clock().now();
 *    std::chrono::duration<double, std::milli> elapsed = end - start;
 *
 *    return 0;
 * }
 * \endcode
 */
class RosClock final : public IClock
{

    rclcpp::Clock m_clock; /**< Ros2 clock */

public:
    /**
     * Get ROS current time
     * @return the current time computed from `rclcpp::Clock::now()`
     * @note `BipedalLocomotion::clock().now().count()` returns a double containing the seconds
     * since epoch.
     */
    std::chrono::nanoseconds now() final;

    /**
     * Blocks the execution of the current thread for at least the specified sleepDuration.
     * @param time duration to sleep
     */
    void sleepFor(const std::chrono::nanoseconds& sleepDuration) final;

    /**
     * Blocks the execution of the current thread until specified sleepTime has been reached.
     * @param time to block until
     */
    void sleepUntil(const std::chrono::nanoseconds& sleepTime) final;

    /**
     * Provides a hint to the implementation to reschedule the execution of threads, allowing other
     * threads to run.
     */
    void yield() final;
};

class RosClockFactory final : public ClockFactory
{
public:
    /**
     * Constructor of the factory.
     * @param argc number of command-line arguments to parse.
     * @param argv array of command-line arguments to parse.
     * @note This function will call `rclcpp::init(argc, argv)`. Please check
     * [here](https://docs.ros.org/en/ros2_packages/rolling/api/rclcpp/generated/function_namespacerclcpp_1a026b2ac505c383735117de5d1679ed80.html?highlight=init)
     * for further details.
     */
    RosClockFactory(int argc, char const* argv[]);

    /**
     * Constructor of the factory.
     * @param args array of command-line arguments to parse.
     * @note This function will call `rclcpp::init(argc, argv)`. Please check
     * [here](https://docs.ros.org/en/ros2_packages/rolling/api/rclcpp/generated/function_namespacerclcpp_1a026b2ac505c383735117de5d1679ed80.html?highlight=init)
     * for further details.
     */
    RosClockFactory(const std::vector<std::string>& args);

    /**
     * Create the ROS clock as a singleton
     * @return the reference to a System::RosClock
     */
    IClock& createClock() final;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_ROS_CLOCK_H
