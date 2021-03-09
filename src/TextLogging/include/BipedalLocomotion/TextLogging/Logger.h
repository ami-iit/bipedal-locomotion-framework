/**
 * @file Logger.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_TEXT_LOGGING_LOGGER_H
#define BIPEDAL_LOCOMOTION_TEXT_LOGGING_LOGGER_H

#include <spdlog/spdlog.h>

namespace BipedalLocomotion
{
namespace TextLogging
{

using Logger = spdlog::logger;

} // namespace TextLogging

/**
 * Get an the instance of the log
 */
TextLogging::Logger* const log();

} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TEXT_LOGGING_LOGGER_H
