/**
 * @file PositionToCurrentController.h
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_JOINT_LEVEL_CONTROLLERS_POSITION_TO_CURRENT_CONTROLLER_H
#define BIPEDAL_LOCOMOTION_BINDINGS_JOINT_LEVEL_CONTROLLERS_POSITION_TO_CURRENT_CONTROLLER_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace JointLevelControllers
{

void CreatePositionToCurrentController(pybind11::module& module);

} // namespace JointLevelControllers
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_JOINT_LEVEL_CONTROLLERS_POSITION_TO_CURRENT_CONTROLLER_H
