/**
 * @file YarpClock.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
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
 * YarpClock implements a the IClock interface using yarp functions.
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
     */
    std::chrono::duration<double> now() final;
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
