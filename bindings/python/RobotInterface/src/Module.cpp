/**
 * @file Module.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/RobotInterface/Module.h>

#include <BipedalLocomotion/bindings/RobotInterface/Polydriver.h>
#include <BipedalLocomotion/bindings/RobotInterface/RobotControl.h>
#include <BipedalLocomotion/bindings/RobotInterface/SensorBridge.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "Robot interface module can be used to communicate with the robot";

    CreatePolyDriver(module);
    CreatePolyDriverDescriptor(module);

    CreateIRobotControl(module);
    CreateYarpRobotControl(module);

    CreateISensorBridge(module);
    CreateYarpSensorBridge(module);
}
} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
