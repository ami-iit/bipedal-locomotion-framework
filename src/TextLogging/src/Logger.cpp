/**
 * @file Logger.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <spdlog/sinks/stdout_color_sinks.h>

#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{

TextLogging::Logger* const log()
{
    auto logger = spdlog::get("blf");

    // if the logger does not exist, create it. spdlog already handle the logger as singleton.
    if (logger == nullptr)
    {
        // create the logger called blf
        auto console = spdlog::stderr_color_mt("blf");

        // get the logger
        logger = spdlog::get("blf");

        // customize the logger
        logger->set_level(spdlog::level::info);
        logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] %^[%l]%$ %v");
    }

    return logger.get();
}

} // namespace BipedalLocomotion
