/**
 * @file Logger.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/TextLogging/LoggerBuilder.h>

namespace BipedalLocomotion
{

TextLogging::Logger* const log()
{
    // m_factory is always initialized.
    assert(BipedalLocomotion::TextLogging::LoggerBuilder::m_factory);
    return BipedalLocomotion::TextLogging::LoggerBuilder::m_factory->createLogger();
}

void TextLogging::setVerbosity(const Verbosity verbosity)
{
    const std::unordered_map<TextLogging::Verbosity, spdlog::level::level_enum> map{
        {TextLogging::Verbosity::Trace, spdlog::level::level_enum::trace},
        {TextLogging::Verbosity::Debug, spdlog::level::level_enum::debug},
        {TextLogging::Verbosity::Info, spdlog::level::level_enum::info},
        {TextLogging::Verbosity::Warn, spdlog::level::level_enum::warn},
        {TextLogging::Verbosity::Err, spdlog::level::level_enum::err},
        {TextLogging::Verbosity::Critical, spdlog::level::level_enum::critical},
        {TextLogging::Verbosity::Off, spdlog::level::level_enum::off},
    };

    if (map.find(verbosity) == map.end())
    {
        log()->error("Failed to change verbosity to level {}", verbosity);
        return;
    }

    log()->set_level(map.at(verbosity));
}

} // namespace BipedalLocomotion
