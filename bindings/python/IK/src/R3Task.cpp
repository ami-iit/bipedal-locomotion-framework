/**
 * @file R3Task.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/IK/R3Task.h>
#include <BipedalLocomotion/System/ITaskControllerManager.h>

#include <BipedalLocomotion/bindings/IK/R3Task.h>

#include <manif/manif.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateR3Task(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;

    py::class_<R3Task,
               std::shared_ptr<R3Task>,
               IKLinearTask,
               BipedalLocomotion::System::ITaskControllerManager>(module, "R3Task")
        .def(py::init())
        .def("set_kin_dyn", &R3Task::setKinDyn, py::arg("kin_dyn"))
        .def("set_set_point",
             &R3Task::setSetPoint,
             py::arg("I_p_F"),
             py::arg("velocity") = Eigen::Vector3d::Zero());
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
