/**
 * @file IClock.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
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
     */
    virtual std::chrono::duration<double> now() = 0;

    /**
     * Blocks the execution of the current thread for at least the specified sleepDuration.
     * @param time duration to sleep
     */
    virtual void sleepFor(const std::chrono::duration<double>& sleepDuration) = 0;

    /**
     * Blocks the execution of the current thread until specified sleepTime has been reached.
     * @param time to block until
     */
    virtual void sleepUntil(const std::chrono::duration<double>& time) = 0;
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
