/**
 * @file IClock.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_ICLOCK_H
#define BIPEDAL_LOCOMOTION_SYSTEM_ICLOCK_H

#include <chrono>

namespace BipedalLocomotion
{
namespace System
{

/**
 * IClock is the interface to the clock. The Clock is considered as a singleton. Please use the
 * ClockBuilder to create a clock. The default clock is the System::StdClock which uses only
 * the std methods. To get the current time you can simply use the following line of code
 * \code{.cpp}
 * BipedalLocomotion::clock().now();
 * \endcode
 */
class IClock
{
protected:

    /**
     * The constructor is protected. Please create the clock with ClockBulder.
     */
    IClock() = default;

public:
    /**
     * Destructor
     */
    virtual ~IClock() = default;

    /**
     * Get the current time
     * @return The current time. The output of the function depends on the concrete implementation.
     * @note `BipedalLocomotion::clock().now().count()` returns a double containing the seconds
     * since epoch.
     */
    virtual std::chrono::nanoseconds now() = 0;

    /**
     * Blocks the execution of the current thread for at least the specified sleepDuration.
     * @param time duration to sleep
     */
    virtual void sleepFor(const std::chrono::nanoseconds& sleepDuration) = 0;

    /**
     * Blocks the execution of the current thread until specified sleepTime has been reached.
     * @param time to block until
     */
    virtual void sleepUntil(const std::chrono::nanoseconds& time) = 0;

    /**
     * Provides a hint to the implementation to reschedule the execution of threads, allowing other
     * threads to run.
     */
    virtual void yield() = 0;
};

/**
 * ClockFactory is an interface that implements the factory paradigm. Please inherit from clock
 * class if you want to build your custom clock.
 */
class ClockFactory
{
public:
    /**
     * Destructor
     */
    virtual ~ClockFactory() = default;

    /**
     * Create a clock
     */
    virtual IClock& createClock() = 0;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_ICLOCK_H
