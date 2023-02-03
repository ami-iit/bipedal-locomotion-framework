/**
 * @file RosModule.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/TextLogging/RosModule.h>
#include <BipedalLocomotion/bindings/TextLogging/RosLogger.h>

namespace BipedalLocomotion::bindings::TextLogging
{
void CreateRosModule(pybind11::module& module)
{
    CreateRosLoggerFactory(module);
}
} // namespace BipedalLocomotion::bindings::TextLogging
