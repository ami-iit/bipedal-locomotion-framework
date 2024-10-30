/**
 * @file StdClock.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_STD_CLOCK_H
#define BIPEDAL_LOCOMOTION_SYSTEM_STD_CLOCK_H

#include <BipedalLocomotion/System/IClock.h>

#include <chrono>

namespace BipedalLocomotion
{
namespace System
{

/**
 * StdClock implements the IClock interface using `<chrono>` from c++std library.
 * The clock can be easily used as follows
 * \code{.cpp}
 * #include <BipedalLocomotion/System/Clock.h>
 *
 *
 * auto start = BipedalLocomotion::clock().now();
 * foo();
 * auto end = BipedalLocomotion::clock().now();
 * std::chrono::duration<double, std::milli> elapsed = end-start;
 * \endcode
 */
class StdClock final : public IClock
{
public:
    /**
     * Get the system current time
     * @return the current time since epoch computed with std::chrono::system_clock
     * @note `BipedalLocomotion::clock().now().count()` returns a double containing the seconds
     * since epoch
     */
    std::chrono::nanoseconds now() final;

    /**
     * Blocks the execution of the current thread for at least the specified sleepDuration.
     * @param time duration to sleep
     * @note std::this_tread::sleep_for() function is used.
     */
    void sleepFor(const std::chrono::nanoseconds& sleepDuration) final;

    /**
     * Blocks the execution of the current thread until specified sleepTime has been reached.
     * @param time to block until
     * @note sleepTime is the duration since epoch
     */
    void sleepUntil(const std::chrono::nanoseconds& sleepTime) final;

    /**
     * Provides a hint to the implementation to reschedule the execution of threads, allowing other
     * threads to run.
     */
    void yield() final;

private:

    /**
     * PrecisionScheduler is a class that allows to set the system timer resolution to the minimum
     * value for higher precision. This class is used only on Windows systems.
     */
    struct PrecisionScheduler
    {
        /**
         * Constructor.
         * It sets the system timer resolution to the minimum value for higher precision.
         * @note Only affects Windows systems.
         */
        PrecisionScheduler();

        /**
         * Destructor.
         * It restores the system timer resolution to the default value.
         * @note Only affects Windows systems.
         */
        ~PrecisionScheduler();
    };

    PrecisionScheduler m_precisionScheduler; /**< PrecisionScheduler object.
                                                  It is used only on Windows systems. */

};

class StdClockFactory final : public ClockFactory
{
public:
    /**
     * Create the std clock as a singleton
     * @return the reference to a System::StdClock
     */
    IClock& createClock() final;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_STD_CLOCK_H
