/**
 * @file YarpClock.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_YARP_CLOCK_H
#define BIPEDAL_LOCOMOTION_SYSTEM_YARP_CLOCK_H

#include <BipedalLocomotion/System/IClock.h>

#include <yarp/os/Network.h>

#include <chrono>

namespace BipedalLocomotion
{
namespace System
{

/**
 * YarpClock implements the IClock interface using yarp functions.
 * The clock can be easily used as follows
 * \code{.cpp}
 * #include <BipedalLocomotion/System/Clock.h>
 * #include <BipedalLocomotion/System/YarpClock.h>
 *
 * // Change the clock
 * BipedalLocomotion::System::ClockBuilder::setFactory(std::make_shared<BipedalLocomotion::System::YarpClockFactory>()));
 *
 * auto start = BipedalLocomotion::clock().now();
 * foo();
 * auto end = BipedalLocomotion::clock().now();
 * std::chrono::duration<double, std::milli> elapsed = end-start;
 * \endcode
 */
class YarpClock final : public IClock
{
    /**
     * A yarp network instance. This automatically call some function required by yarp::os::Time
     */
    yarp::os::Network m_network;

public:
    /**
     * Get YARP current time
     * @return the current time computed from `yarp::os::Time::now()`
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

class YarpClockFactory final : public ClockFactory
{
public:
    /**
     * Create the YARP clock as a singleton
     * @return the reference to a System::YarpClock
     */
    IClock& createClock() final;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_YARP_CLOCK_H
