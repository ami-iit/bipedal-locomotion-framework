/**
 * @file LoggerBuilder.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iostream>

#include <BipedalLocomotion/TextLogging/LoggerBuilder.h>

using namespace BipedalLocomotion::TextLogging;

bool LoggerBuilder::setFactory(std::shared_ptr<LoggerFactory> factory)
{
    constexpr auto logPrefix = "[LoggerBuilder::setFactory]";
    if (factory == nullptr)
    {
        // logger cannot be used here
        std::cerr << logPrefix << " The factory is not valid." << std::endl;
        return false;
    }

    m_factory = factory;
    return true;
}
