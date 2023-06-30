/**
 * @file Module.h
 * @authors Carlotta Sartore
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_REDUCED_MODEL_CONTROLLERS_MODULE_H
#define BIPEDAL_LOCOMOTION_BINDINGS_REDUCED_MODEL_CONTROLLERS_MODULE_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ReducedModelControllers
{

void CreateModule(pybind11::module& module);

} // namespace Contacts
} // namespace bindings
} // namespace ReducedModelControllers

#endif // BIPEDAL_LOCOMOTION_BINDINGS_REDUCED_MODEL_CONTROLLERS_MODULE_H
