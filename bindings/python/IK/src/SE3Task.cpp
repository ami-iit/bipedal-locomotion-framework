/**
 * @file SE3Task.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <manif/impl/se3/SE3.h>
#include <pybind11/detail/common.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/System/ITaskControllerManager.h>

#include <BipedalLocomotion/bindings/IK/SE3Task.h>

#include <manif/manif.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateSE3Task(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;

    py::class_<SE3Task,
               std::shared_ptr<SE3Task>,
               IKLinearTask,
               BipedalLocomotion::System::ITaskControllerManager>(module, "SE3Task")
        .def(py::init())
        .def("set_kin_dyn", &SE3Task::setKinDyn, py::arg("kin_dyn"))
        .def("set_set_point",
             &SE3Task::setSetPoint,
             py::arg("I_H_F"),
             py::arg("mixed_velocity") = manif::SE3d::Tangent::Zero())
        .def("set_feedback",
             py::overload_cast<const manif::SE3d&>(&SE3Task::setFeedback),
             py::arg("I_H_F"))
        .def("set_feedback",
             py::overload_cast<const manif::SE3d::Translation&>(&SE3Task::setFeedback),
             py::arg("I_p_F"))
        .def("set_feedback",
             py::overload_cast<const manif::SO3d&>(&SE3Task::setFeedback),
             py::arg("I_R_F"));
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
