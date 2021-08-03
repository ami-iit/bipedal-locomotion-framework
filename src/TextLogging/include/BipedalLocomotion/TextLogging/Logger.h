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

} // namespace TextLogging

/**
 * Get an the instance of the log
 */
TextLogging::Logger* const log();

} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TEXT_LOGGING_LOGGER_H
