/**
 * @file RosLogger.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TEXT_LOGGING_ROS_LOGGER_H
#define BIPEDAL_LOCOMOTION_TEXT_LOGGING_ROS_LOGGER_H

#include <memory>
#include <mutex>
#include <string>

#include <spdlog/common.h>
#include <spdlog/sinks/base_sink.h>

#include <BipedalLocomotion/TextLogging/Logger.h>

#include <rclcpp/logging.hpp>

namespace BipedalLocomotion
{
namespace TextLogging
{

namespace sinks
{

template <typename Mutex> class RosSink : public spdlog::sinks::base_sink<Mutex>
{
public:
    RosSink(const rclcpp::Logger& rosLogger)
        : m_rosLogger(rosLogger)
    {
    }

private:
    rclcpp::Logger m_rosLogger;

protected:
    void sink_it_(const spdlog::details::log_msg& msg) override
    {
        // log_msg is a struct containing the log entry info like level, timestamp, thread id etc.
        // msg.raw contains pre formatted log
        spdlog::memory_buf_t formatted;

        spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);

        if (msg.level == spdlog::level::level_enum::trace)
        {
            RCLCPP_INFO(m_rosLogger, formatted.data());

        } else if (msg.level == spdlog::level::level_enum::debug)
        {
            RCLCPP_DEBUG(m_rosLogger, formatted.data());
        } else if (msg.level == spdlog::level::level_enum::info)
        {
            RCLCPP_INFO(m_rosLogger, formatted.data());
        } else if (msg.level == spdlog::level::level_enum::warn)
        {
            RCLCPP_WARN(m_rosLogger, formatted.data());
        } else if (msg.level == spdlog::level::level_enum::err)
        {
            RCLCPP_ERROR(m_rosLogger, formatted.data());
        } else
        {
            RCLCPP_FATAL(m_rosLogger, formatted.data());
        }
    }

    void flush_() override
    {
    }
};

using RosSink_mt = RosSink<std::mutex>;
} // namespace sinks

/**
 * RosLoggerFactory implements the factory you should use to enable the sink using ros.
 * The ROS logger can be easily used as follows
 * \code{.cpp}
 * #include <BipedalLocomotion/TextLogging/Logger.h>
 * #include <BipedalLocomotion/TextLogging/RosLogger.h>
 * #include <BipedalLocomotion/TextLogging/LoggerBuilder.h>
 *
 * // Change the logger
 * BipedalLocomotion::TextLogging::LoggerBuilder::setFactory(std::make_shared<BipedalLocomotion::TextLogging::RosLoggerFactory>()));
 *
 * BipedalLocomotion::log()->info("My info");
 * \endcode
 */
class RosLoggerFactory final : public LoggerFactory
{
public:
    /**
     * Construct a new RosLoggerFactory object
     * @param name the name of the logger which will be used inside the formatted messages
     */
    RosLoggerFactory(const std::string_view& name = "blf");

    /**
     * Construct a new RosLoggerFactory object
     * @param name the name of the logger which will be used inside the formatted messages
     */
    RosLoggerFactory(const rclcpp::Logger& logger);

    /**
     * Create the ROSLogger as a singleton
     * @return the pointer to TextLogging::Logger that streams the output using ROS
     */
    std::shared_ptr<TextLogging::Logger> const createLogger() final;

private:
    const std::string m_name; /** The name of the logger */
    const rclcpp::Logger m_rosLogger; /** Associated ros logger */
};

} // namespace TextLogging
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TEXT_LOGGING_ROS_LOGGER_H
