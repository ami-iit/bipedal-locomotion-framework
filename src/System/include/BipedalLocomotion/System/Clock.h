/**
 * @file Clock.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_CLOCK_H
#define BIPEDAL_LOCOMOTION_SYSTEM_CLOCK_H

#include <memory>

#include <BipedalLocomotion/System/IClock.h>
#include <BipedalLocomotion/System/StdClock.h>

namespace BipedalLocomotion
{

/**
 * Get the clock singleton.
 */
System::IClock& clock();

namespace System
{

class ClockBuilder
{
    /**
     * Pointer to factory used to build the clock
     */
    inline static std::shared_ptr<ClockFactory> m_factory{std::make_shared<StdClockFactory>()};
    inline static bool m_clockAlreadyCalledOnce{false}; /**< True if the clock() has been already
                                                           called once. If True it will not be
                                                           possible to set a new Factory */

public:
    /**
    * Set a custom factory.
    * @param factory. A pointer to an existing factory.
    * @return True in case success, false otherwise.
    */
    static bool setFactory(std::shared_ptr<ClockFactory> factory);

    friend IClock& ::BipedalLocomotion::clock();
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_CLOCK_H
