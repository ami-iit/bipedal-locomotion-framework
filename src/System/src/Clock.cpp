/**
 * @file Clock.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

BipedalLocomotion::System::IClock& BipedalLocomotion::clock()
{
    // m_factory is always initialized.
    assert(BipedalLocomotion::System::ClockBuilder::m_factory);
    return BipedalLocomotion::System::ClockBuilder::m_factory->createClock();
}

bool BipedalLocomotion::System::ClockBuilder::setFactory(std::shared_ptr<ClockFactory> factory)
{
    constexpr auto logPrefix = "[ClockBuilder::setFactory]";
    if (factory == nullptr)
    {
        log()->error("{} The factory is not valid.", logPrefix);
        return false;
    }

    m_factory = factory;
    return true;
}
