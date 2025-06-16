/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/JointLevelControllers/Module.h>
#include <BipedalLocomotion/bindings/JointLevelControllers/PositionToCurrentController.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace JointLevelControllers
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "Controllers for robot joints";

    CreatePositionToCurrentController(module);
}
} // namespace JointLevelControllers
} // namespace bindings
} // namespace BipedalLocomotion
