/**
 * @file DefaultLogger.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <spdlog/sinks/stdout_color_sinks.h>

#include <BipedalLocomotion/TextLogging/DefaultLogger.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{

std::shared_ptr<TextLogging::Logger> _createLogger(const std::string& name)
{
    auto logger = spdlog::get(name);

    // if the logger called blf already exist. If it does not exist it is created.
    if (logger == nullptr)
    {
        // spdlog already handle the logger as singleton create the logger called blf
        auto console = spdlog::stdout_color_mt(name);

        // get the logger
        logger = spdlog::get(name);

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

TextLogging::DefaultLoggerFactory::DefaultLoggerFactory(const std::string_view& name)
    : m_name(name)
{
}

std::shared_ptr<TextLogging::Logger> const TextLogging::DefaultLoggerFactory::createLogger()
{
    // Since the oobject is static the memory is not deallocated
    static std::shared_ptr<TextLogging::Logger> logger(_createLogger(m_name));

    // the logger exist because loggerCreation is called.
    return logger;
}
} // namespace BipedalLocomotion
