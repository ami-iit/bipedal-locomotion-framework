/**
 * @file IRobotControl.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <optional>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/RobotInterface/IRobotControl.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <BipedalLocomotion/bindings/RobotInterface/IRobotControl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{

void CreateIRobotControl(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::RobotInterface;

    py::class_<IRobotControl> iRobotControl(module, "IRobotControl");
    py::enum_<IRobotControl::ControlMode>(iRobotControl, "ControlMode")
        .value("Position", IRobotControl::ControlMode::Position)
        .value("PositionDirect", IRobotControl::ControlMode::PositionDirect)
        .value("Velocity", IRobotControl::ControlMode::Velocity)
        .value("Torque", IRobotControl::ControlMode::Torque)
        .value("PWM", IRobotControl::ControlMode::PWM)
        .value("Current", IRobotControl::ControlMode::Current)
        .value("Idle", IRobotControl::ControlMode::Idle)
        .value("Unknown", IRobotControl::ControlMode::Unknown)
        .export_values();
}

} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
