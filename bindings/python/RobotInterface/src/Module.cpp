/**
 * @file Module.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/RobotInterface/IRobotControl.h>
#include <BipedalLocomotion/bindings/RobotInterface/ISensorBridge.h>
#include <BipedalLocomotion/bindings/RobotInterface/Module.h>
#include <BipedalLocomotion/bindings/RobotInterface/PositionToCurrentController.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "Robot interface module can be used to communicate with the robot";

    CreatePositionToCurrentController(module);
    CreateIRobotControl(module);
    CreateISensorBridge(module);
}
} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
