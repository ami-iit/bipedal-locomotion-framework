/**
 * @file Logger.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/TextLogging/LoggerBuilder.h>

namespace BipedalLocomotion
{

std::shared_ptr<TextLogging::Logger> const log()
{
    // m_factory is always initialized.
    assert(BipedalLocomotion::TextLogging::LoggerBuilder::m_factory);
    return BipedalLocomotion::TextLogging::LoggerBuilder::m_factory->createLogger();
}

void TextLogging::setVerbosity(const Verbosity verbosity)
{
    // get the verbosity underling value and convert it in spdlog enum type
    const auto value = static_cast<std::underlying_type<Verbosity>::type>(verbosity);
    log()->set_level(static_cast<spdlog::level::level_enum>(value));
}

} // namespace BipedalLocomotion
