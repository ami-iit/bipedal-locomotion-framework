/**
 * @file UnicyclePlanner.h
 * @authors Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_PLANNERS_UNICYCLE_PLANNER_H
#define BIPEDAL_LOCOMOTION_BINDINGS_PLANNERS_UNICYCLE_PLANNER_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion::bindings::Planners
{
void CreateUnicyclePlanner(pybind11::module& module);
} // namespace BipedalLocomotion::bindings::Planners

#endif // BIPEDAL_LOCOMOTION_BINDINGS_PLANNERS_UNICYCLE_PLANNER_H
