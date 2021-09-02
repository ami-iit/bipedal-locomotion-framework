/**
 * @file Logger.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_TEXT_LOGGING_LOGGER_H
#define BIPEDAL_LOCOMOTION_TEXT_LOGGING_LOGGER_H

// Required to print Eigen::Vectors (https://github.com/gabime/spdlog/issues/1638)
#include <spdlog/fmt/ostr.h>

#include <spdlog/spdlog.h>

namespace BipedalLocomotion
{
namespace TextLogging
{

using Logger = spdlog::logger;
} // namespace TextLogging
} // namespace BipedalLocomotion

namespace BipedalLocomotion
{
/**
 * Get an the instance of the log
 */
TextLogging::Logger* const log();

} // namespace BipedalLocomotion

namespace BipedalLocomotion
{
namespace TextLogging
{
enum class Verbosity
{
    Trace,
    Debug,
    Info,
    Warn,
    Err,
    Critical,
    Off,
};

/**
 * Set the logger verbosity.
 *
 * @param verbosity The desired verbosity level.
 */
void setVerbosity(const TextLogging::Verbosity verbosity);

/**
 * LoggerFactory is an interface that implements the factory paradigm. Please inherit from
 * LoggerFactory class if you want to build your custom Logger.
 */
class LoggerFactory
{
public:
    /**
     * Destructor
     */
    virtual ~LoggerFactory() = default;

    /**
     * Create a Logger
     */
    virtual Logger* const createLogger() = 0;
};

} // namespace TextLogging
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TEXT_LOGGING_LOGGER_H
