/**
 * @file UnicycleModule.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_PLANNERS_UNICYCLE_MODULE_H
#define BIPEDAL_LOCOMOTION_BINDINGS_PLANNERS_UNICYCLE_MODULE_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Planners
{

void CreateUnicycleModule(pybind11::module& module);

} // namespace Planners
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_PLANNERS_UNICYCLE_MODULE_H
