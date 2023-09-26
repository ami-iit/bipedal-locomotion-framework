/**
 * @file GravityTask.h
 * @authors Stefano Dafarra
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_IK_GRAVITY_TASK_H
#define BIPEDAL_LOCOMOTION_BINDINGS_IK_GRAVITY_TASK_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateGravityTask(pybind11::module& module);

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_IK_GRAVITY_TASK_H
