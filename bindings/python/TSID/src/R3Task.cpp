/**
 * @file R3Task.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/System/ITaskControllerManager.h>
#include <BipedalLocomotion/TSID/R3Task.h>
#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

#include <BipedalLocomotion/bindings/System/LinearTask.h>
#include <BipedalLocomotion/bindings/TSID/R3Task.h>
#include <BipedalLocomotion/bindings/TSID/TSIDLinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateR3Task(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;

    py::class_<R3Task,
               std::shared_ptr<R3Task>, //
               TSIDLinearTask,
               BipedalLocomotion::System::ITaskControllerManager>(module, "R3Task")
        .def(py::init())
        .def("set_kin_dyn",
             BipedalLocomotion::bindings::System::setKinDyn<R3Task>,
             py::arg("kin_dyn"))
        .def("set_set_point",
             &R3Task::setSetPoint,
             py::arg("I_p_F"),
             py::arg("linear_velocity") = Eigen::Vector3d::Zero(),
             py::arg("linear_acceleration") = Eigen::Vector3d::Zero())
        .def("get_controller_output", &R3Task::getControllerOutput);
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
