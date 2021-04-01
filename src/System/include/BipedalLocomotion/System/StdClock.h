/**
 * @file StdClock.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
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
 * StdClock implements a the IClock interface using.
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
     */
    std::chrono::duration<double> now() final;
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
