/**
 * @file CoMZMPController.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/SimplifiedModelControllers/CoMZMPController.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace SimplifiedModelControllers
{

void CreateCoMZMPController(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::SimplifiedModelControllers;
    using namespace BipedalLocomotion::System;

    py::class_<CoMZMPControllerInput>(module, "CoMZMPControllerInput")
        .def(py::init())
        .def_readwrite("desired_CoM_position", &CoMZMPControllerInput::desiredCoMPosition)
        .def_readwrite("desired_CoM_velocity", &CoMZMPControllerInput::desiredCoMVelocity)
        .def_readwrite("desired_ZMP_position", &CoMZMPControllerInput::desiredZMPPosition)
        .def_readwrite("CoM_position", &CoMZMPControllerInput::CoMPosition)
        .def_readwrite("ZMP_position", &CoMZMPControllerInput::ZMPPosition)
        .def_readwrite("angle", &CoMZMPControllerInput::angle);

    BipedalLocomotion::bindings::System::CreateAdvanceable<CoMZMPControllerInput,
                                                           Eigen::Vector2d>(module,
                                                                            "CoMZMPController");
    py::class_<CoMZMPController, //
               Advanceable<CoMZMPControllerInput, Eigen::Vector2d>>(module, "CoMZMPController")
        .def(py::init())
        .def("set_set_point",
             &CoMZMPController::setSetPoint,
             py::arg("CoM_velocity"),
             py::arg("CoM_position"),
             py::arg("ZMP_position"))
        .def("set_feedback",
             py::overload_cast<Eigen::Ref<const Eigen::Vector2d>,
                               Eigen::Ref<const Eigen::Vector2d>,
                               const manif::SO2d&>(&CoMZMPController::setFeedback),
             py::arg("CoM_position"),
             py::arg("ZMP_position"),
             py::arg("I_R_B"))
        .def("set_feedback",
             py::overload_cast<Eigen::Ref<const Eigen::Vector2d>,
                               Eigen::Ref<const Eigen::Vector2d>,
                               const double>(&CoMZMPController::setFeedback),
             py::arg("CoM_position"),
             py::arg("ZMP_position"),
             py::arg("angle"));
}

} // namespace SimplifiedModelControllers
} // namespace bindings
} // namespace BipedalLocomotion
