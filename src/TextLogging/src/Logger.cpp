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

std::shared_ptr<TextLogging::Logger> loggerCreation()
{
    auto logger = spdlog::get("blf");

    // if the logger called blf already exist. If it does not exist it is created.
    if (logger == nullptr)
    {
        // spdlog already handle the logger as singleton create the logger called blf
        auto console = spdlog::stdout_color_mt("blf");

        // get the logger
        logger = spdlog::get("blf");

        // if the project is compiled in debug the level of spdlog is set in debug
#ifdef NDEBUG
        logger->set_level(spdlog::level::info);
#else
        logger->set_level(spdlog::level::debug);
#endif // NDEBUG

        // set the custom pattern
        logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [thread: %t] [%n] %^[%l]%$ %v");
    }
    return logger;
}

TextLogging::Logger* const log()
{
    // Since the oobject is static the memory is not deallocated
    static std::shared_ptr<TextLogging::Logger> logger(loggerCreation());

    // the logger exist because loggerCreation is called.
    return logger.get();
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
