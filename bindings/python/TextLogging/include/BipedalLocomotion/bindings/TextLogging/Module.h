/**
 * @file Module.h
 * @authors Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_TEXT_LOGGING_MODULE_H
#define BIPEDAL_LOCOMOTION_BINDINGS_TEXT_LOGGING_MODULE_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion::bindings::TextLogging
{
void CreateModule(pybind11::module& module);
} // namespace BipedalLocomotion::bindings::TextLogging

#endif // BIPEDAL_LOCOMOTION_BINDINGS_TEXT_LOGGING_MODULE_H
