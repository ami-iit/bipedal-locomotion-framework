/**
 * @file R3Task.h
 * @authors Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_TSID_R3_TASK_H
#define BIPEDAL_LOCOMOTION_BINDINGS_TSID_R3_TASK_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateR3Task(pybind11::module& module);

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_TSID_R3_TASK_H
