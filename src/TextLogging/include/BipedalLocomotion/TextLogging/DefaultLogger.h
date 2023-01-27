/**
 * @file DefaultLogger.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TEXT_LOGGING_DEFAULT_LOGGER_FACTORY_H
#define BIPEDAL_LOCOMOTION_TEXT_LOGGING_DEFAULT_LOGGER_FACTORY_H

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <string_view>

namespace BipedalLocomotion
{
namespace TextLogging
{

class DefaultLoggerFactory final : public LoggerFactory
{
public:
    /**
     * Construct a new DefaultLoggerFactory object
     * @param name the name of the logger which will be used inside the formatted messages
     */
    DefaultLoggerFactory(const std::string_view& name = "blf");

    /**
     * Create the std clock as a singleton
     * @return the reference to a System::StdClock
     */
    std::shared_ptr<Logger> const createLogger() final;

private:
    const std::string m_name; /** The name of the logger */
};

} // namespace TextLogging
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TEXT_LOGGING_DEFAULT_LOGGER_FACTORY_H
