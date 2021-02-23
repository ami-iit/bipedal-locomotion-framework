/**
 * @file RobotInterface.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <yarp/dev/PolyDriver.h>

#include "BipedalLocomotion/RobotInterface/IRobotControl.h"
#include "BipedalLocomotion/RobotInterface/YarpHelper.h"
#include "BipedalLocomotion/RobotInterface/YarpRobotControl.h"

#include "bipedal_locomotion_framework.h"

void BipedalLocomotion::bindings::CreatePolyDriver(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace yarp::dev;
    py::class_<PolyDriver, std::shared_ptr<PolyDriver>>(module, "PolyDriver");
}

void BipedalLocomotion::bindings::CreatePolyDriverDescriptor(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::RobotInterface;
    using namespace BipedalLocomotion::ParametersHandler;

    py::class_<PolyDriverDescriptor>(module, "PolyDriverDescriptor")
        .def(py::init())
        .def_readwrite("key", &PolyDriverDescriptor::key)
        .def_readwrite("poly", &PolyDriverDescriptor::poly)
        .def("is_valid", &PolyDriverDescriptor::isValid);

    module.def(
        "construct_remote_control_board_remapper",
        [](std::shared_ptr<IParametersHandler> handler) -> PolyDriverDescriptor {
            return constructRemoteControlBoardRemapper(handler);
        },
        py::arg("handler"));

    module.def(
        "construct_generic_sensor_client",
        [](std::shared_ptr<IParametersHandler> handler) -> PolyDriverDescriptor {
            return constructGenericSensorClient(handler);
        },
        py::arg("handler"));
}

void BipedalLocomotion::bindings::CreateIRobotControl(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::RobotInterface;

    py::class_<IRobotControl> iRobotControl(module, "IRobotControl");
    py::enum_<IRobotControl::ControlMode>(iRobotControl, "ControlMode")
        .value("Position", IRobotControl::ControlMode::Position)
        .value("PositionDirect", IRobotControl::ControlMode::PositionDirect)
        .value("Velocity", IRobotControl::ControlMode::Velocity)
        .value("Torque", IRobotControl::ControlMode::Torque)
        .value("Idle", IRobotControl::ControlMode::Idle)
        .value("Unknown", IRobotControl::ControlMode::Unknown)
        .export_values();
}

void BipedalLocomotion::bindings::CreateYarpRobotControl(pybind11::module& module)
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
                               const std::vector<IRobotControl::ControlMode>&>(
                 &YarpRobotControl::setReferences),
             py::arg("joints_value"),
             py::arg("control_modes"))
        .def("set_references",
             py::overload_cast<Eigen::Ref<const Eigen::VectorXd>, const IRobotControl::ControlMode&>(
                 &YarpRobotControl::setReferences),
             py::arg("joints_value"),
             py::arg("control_mode"))
        .def("get_joint_list", &YarpRobotControl::getJointList);
}
