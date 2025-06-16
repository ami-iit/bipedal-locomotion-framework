/**
 * @file Module.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/RobotInterface/YarpModule.h>

#include <BipedalLocomotion/bindings/RobotInterface/Polydriver.h>
#include <BipedalLocomotion/bindings/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/bindings/RobotInterface/YarpSensorBridge.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{
void CreateYarpModule(pybind11::module& module)
{
    CreatePolyDriver(module);
    CreatePolyDriverDescriptor(module);

    CreateYarpRobotControl(module);

    CreateYarpSensorBridge(module);
}
} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
