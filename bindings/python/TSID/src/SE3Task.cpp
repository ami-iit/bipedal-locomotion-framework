/**
 * @file SE3Task.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/System/ITaskControllerManager.h>
#include <BipedalLocomotion/TSID/SE3Task.h>
#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

#include <BipedalLocomotion/bindings/TSID/SE3Task.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateSE3Task(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;

    py::class_<SE3Task, std::shared_ptr<SE3Task>, TSIDLinearTask,
	       BipedalLocomotion::System::ITaskControllerManager>(module, "SE3Task")
        .def(py::init())
        .def("set_kin_dyn", &SE3Task::setKinDyn, py::arg("kin_dyn"))
        .def("set_set_point",
             &SE3Task::setSetPoint,
             py::arg("I_H_F"),
             py::arg("mixed_velocity"),
             py::arg("mixed_acceleration"));
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
