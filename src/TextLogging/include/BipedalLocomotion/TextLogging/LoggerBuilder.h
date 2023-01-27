/**
 * @file LoggerBuilder.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TEXT_LOGGING_LOGGER_BUILDER_H
#define BIPEDAL_LOCOMOTION_TEXT_LOGGING_LOGGER_BUILDER_H

#include <memory>

#include <BipedalLocomotion/TextLogging/DefaultLogger.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
namespace TextLogging
{

/**
 * LoggerBuilder is a class that implements the Builder paradigm. You can use the LoggerBuilder at
 * the beginning of your application to set a different sink for spdlog. For instance, you can use
 * the following example to enable the TextLogging with YARP
 * \code{.cpp}
 * #include <BipedalLocomotion/TextLogging/Logger.h>
 * #include <BipedalLocomotion/TextLogging/YarpLogger.h>
 * #include <BipedalLocomotion/TextLogging/LoggerBuilder.h>
 *
 * // Change the logger
 * BipedalLocomotion::TextLogging::LoggerBuilder::setFactory(std::make_shared<BipedalLocomotion::TextLogging::YarpLoggerFactory>());
 *
 * BipedalLocomotion::log()->info("My info");
 * \endcode
 */
class LoggerBuilder
{
    /**
     * Pointer to factory used to build the clock
     */
    inline static std::shared_ptr<LoggerFactory> m_factory{
        std::make_shared<DefaultLoggerFactory>()};

public:
    /**
     * Set a custom factory.
     * @param factory. A pointer to an existing factory.
     * @return True in case success, false otherwise.
     */
    static bool setFactory(std::shared_ptr<LoggerFactory> factory);

    friend std::shared_ptr<Logger> const ::BipedalLocomotion::log();
};

}; // namespace TextLogging
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TEXT_LOGGING_LOGGER_BUILDER_H
