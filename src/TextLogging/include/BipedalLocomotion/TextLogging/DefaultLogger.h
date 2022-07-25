/**
 * @file DefaultLogger.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TEXT_LOGGING_DEFAULT_LOGGER_FACTORY_H
#define BIPEDAL_LOCOMOTION_TEXT_LOGGING_DEFAULT_LOGGER_FACTORY_H

#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
namespace TextLogging
{

class DefaultLoggerFactory final : public LoggerFactory
{
public:
    /**
     * Create the std clock as a singleton
     * @return the reference to a System::StdClock
     */
    Logger* const createLogger() final;
};

} // namespace TextLogging
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TEXT_LOGGING_DEFAULT_LOGGER_FACTORY_H
