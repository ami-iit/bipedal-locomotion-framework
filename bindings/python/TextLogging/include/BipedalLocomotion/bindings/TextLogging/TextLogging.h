/**
 * @file TextLogging.h
 * @authors Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_TEXT_LOGGING_H
#define BIPEDAL_LOCOMOTION_BINDINGS_TEXT_LOGGING_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion::bindings::TextLogging
{
void CreateTextLogging(pybind11::module& module);
} // namespace BipedalLocomotion::bindings::TextLogging

#endif // BIPEDAL_LOCOMOTION_BINDINGS_TEXT_LOGGING_H
