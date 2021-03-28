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

std::once_flag initInstanceFlag;

void loggerCreation()
{
    // if the logger does not exist, create it. spdlog already handle the logger as singleton.
    // create the logger called blf
    auto console = spdlog::stderr_color_mt("blf");

    // get the logger
    auto logger = spdlog::get("blf");

    // customize the logger
    logger->set_level(spdlog::level::info);
    logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [thread: %t] [%n] %^[%l]%$ %v");
}

TextLogging::Logger* const log()
{
    std::call_once(initInstanceFlag, loggerCreation);

    // the logger exist because loggerCreation is called once.
    return spdlog::get("blf").get();
}

} // namespace BipedalLocomotion
