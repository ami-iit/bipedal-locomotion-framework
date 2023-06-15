/**
 * @file Module.cpp
 * @authors Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include "BipedalLocomotion/bindings/TextLogging/Module.h"
#include "BipedalLocomotion/bindings/TextLogging/TextLogging.h"

namespace BipedalLocomotion::bindings::TextLogging
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "Text logging module";
    CreateTextLogging(module);
}
} // namespace BipedalLocomotion::bindings::TextLogging
