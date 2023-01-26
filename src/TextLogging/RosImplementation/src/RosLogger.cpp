/**
 * @file RosLogger.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <mutex>

#include <BipedalLocomotion/TextLogging/RosLogger.h>

#include <rclcpp/rclcpp.hpp>

namespace BipedalLocomotion
{

template <typename Factory = spdlog::synchronous_factory>
inline std::shared_ptr<TextLogging::Logger> RosSink_mt(const std::string& loggerName)
{
    rclcpp::Logger logger =  rclcpp::get_logger(loggerName);
    return Factory::template create<TextLogging::sinks::RosSink_mt>(loggerName, std::move(logger));
}

std::shared_ptr<TextLogging::Logger> _createLogger(const std::string& name)
{
    auto logger = spdlog::get(name);

    // if the logger called blf already exist. If it does not exist it is created.
    if (logger == nullptr)
    {
        // spdlog already handle the logger as singleton create the logger called blf
        auto console = RosSink_mt(name);

        // get the logger
        logger = spdlog::get(name);

        // if the project is compiled in debug the level of spdlog is set in debug
#ifdef NDEBUG
        logger->set_level(spdlog::level::info);
#else
        logger->set_level(spdlog::level::debug);
#endif // NDEBUG

        // set the custom pattern
        logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [thread: %t] [%n] %v");
    }
    return logger;
}

TextLogging::RosLoggerFactory::RosLoggerFactory(const std::string_view& name)
    : m_name{name}
{
}

TextLogging::Logger* const TextLogging::RosLoggerFactory::createLogger()
{
    // Since the oobject is static the memory is not deallocated
    static std::shared_ptr<TextLogging::Logger> logger(_createLogger(m_name));

    // the logger exist because loggerCreation is called.
    return logger.get();
}

} // namespace BipedalLocomotion
