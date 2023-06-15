/**
 * @file Clock.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

BipedalLocomotion::System::IClock& BipedalLocomotion::clock()
{
    // m_factory is always initialized.
    assert(BipedalLocomotion::System::ClockBuilder::m_factory);
    BipedalLocomotion::System::ClockBuilder::m_clockAlreadyCalledOnce = true;
    return BipedalLocomotion::System::ClockBuilder::m_factory->createClock();
}

bool BipedalLocomotion::System::ClockBuilder::setFactory(std::shared_ptr<ClockFactory> factory)
{
    constexpr auto logPrefix = "[ClockBuilder::setFactory]";

    if (m_clockAlreadyCalledOnce)
    {
        log()->error("{} The clock has been already called. Since the clock() returns a singleton "
                     "it is not possible to set the factory anymore. Please call 'setFactory()' at "
                     "the beginning of your application to avoid this problem.",
                     logPrefix);
        return false;
    }

    if (factory == nullptr)
    {
        log()->error("{} The factory is not valid.", logPrefix);
        return false;
    }

    m_factory = factory;
    return true;
}
