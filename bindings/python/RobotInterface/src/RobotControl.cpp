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
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <BipedalLocomotion/bindings/RobotInterface/RobotControl.h>

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
        .value("Idle", IRobotControl::ControlMode::Idle)
        .value("Unknown", IRobotControl::ControlMode::Unknown)
        .export_values();
}

void CreateYarpRobotControl(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::ParametersHandler;
    using namespace BipedalLocomotion::RobotInterface;

    py::class_<YarpRobotControl, IRobotControl>(module, "YarpRobotControl")
        .def(py::init())
        .def(
            "initialize",
            [](YarpRobotControl& impl, std::shared_ptr<IParametersHandler> handler) -> bool {
                return impl.initialize(handler);
            },
            py::arg("handler"))
        .def("set_driver", &YarpRobotControl::setDriver, py::arg("driver"))
        .def("check_motion_done",
             [](YarpRobotControl& impl) {
                 bool motionDone{false};
                 bool isTimeExpired{false};
                 std::vector<std::pair<std::string, double>> info;
                 bool isOk = impl.checkMotionDone(motionDone, isTimeExpired, info);

                 return std::make_tuple(isOk, motionDone, isTimeExpired, info);
             })
        .def("set_references",
             py::overload_cast<Eigen::Ref<const Eigen::VectorXd>,
                               const std::vector<IRobotControl::ControlMode>&,
                               std::optional<Eigen::Ref<const Eigen::VectorXd>>>(
                 &YarpRobotControl::setReferences),
             py::arg("desired_joints_value"),
             py::arg("control_modes"),
             py::arg("current_joint_values") = std::nullopt)
        .def("set_references",
             py::overload_cast<Eigen::Ref<const Eigen::VectorXd>,
                               const IRobotControl::ControlMode&,
                               std::optional<Eigen::Ref<const Eigen::VectorXd>>>(
                 &YarpRobotControl::setReferences),
             py::arg("desired_joints_value"),
             py::arg("control_mode"),
             py::arg("current_joint_values") = std::nullopt)
        .def("set_control_mode",
             py::overload_cast<const std::vector<IRobotControl::ControlMode>&>(
                 &YarpRobotControl::setControlMode),
             py::arg("control_modes"))
        .def("set_control_mode",
             py::overload_cast<const IRobotControl::ControlMode&>(
                 &YarpRobotControl::setControlMode),
             py::arg("control_mode"))
        .def("get_joint_list", &YarpRobotControl::getJointList)
        .def("is_valid", &YarpRobotControl::isValid)
        .def("get_joint_limits", [](YarpRobotControl& impl) {
            Eigen::VectorXd lowerLimits, upperLimits;
            lowerLimits.resize(impl.getJointList().size());
            upperLimits.resize(impl.getJointList().size());
            bool ok = impl.getJointLimits(lowerLimits, upperLimits);
            return std::make_tuple(ok, lowerLimits, upperLimits);
        });
}

} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
