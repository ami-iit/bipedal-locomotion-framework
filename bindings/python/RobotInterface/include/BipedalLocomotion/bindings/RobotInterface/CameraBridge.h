/**
 * @file CameraBridge.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_ROBOT_INTERFACE_CAMERA_BRIDGE_H
#define BIPEDAL_LOCOMOTION_BINDINGS_ROBOT_INTERFACE_CAMERA_BRIDGE_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{

void CreateICameraBridge(pybind11::module& module);
void CreateYarpCameraBridge(pybind11::module& module);

} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_ROBOT_INTERFACE_CAMERA_BRIDGE_H
