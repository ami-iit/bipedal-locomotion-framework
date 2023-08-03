/**
 * @file Module.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_SIMPLIFIED_MODEL_CONTROLLERS_MODULE_H
#define BIPEDAL_LOCOMOTION_BINDINGS_SIMPLIFIED_MODEL_CONTROLLERS_MODULE_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace SimplifiedModelControllers
{

void CreateModule(pybind11::module& module);

} // namespace Contacts
} // namespace bindings
} // namespace SimplifiedModelControllers

#endif // BIPEDAL_LOCOMOTION_BINDINGS_SIMPLIFIED_MODEL_CONTROLLERS_MODULE_H
