/**
 * @file YarpModule.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/TextLogging/YarpModule.h>
#include <BipedalLocomotion/bindings/TextLogging/YarpLogger.h>

namespace BipedalLocomotion::bindings::TextLogging
{
void CreateYarpModule(pybind11::module& module)
{
    CreateYarpLoggerFactory(module);
}
} // namespace BipedalLocomotion::bindings::TextLogging
