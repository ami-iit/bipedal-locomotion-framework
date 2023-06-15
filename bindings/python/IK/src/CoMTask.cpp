/**
 * @file CoMTask.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/CoMTask.h>

#include <BipedalLocomotion/bindings/IK/CoMTask.h>
#include <BipedalLocomotion/bindings/IK/IKLinearTask.h>
#include <BipedalLocomotion/bindings/System/LinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateCoMTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;

    py::class_<CoMTask, std::shared_ptr<CoMTask>, IKLinearTask>(module, "CoMTask")
        .def(py::init())
        .def("set_set_point", &CoMTask::setSetPoint, py::arg("position"), py::arg("velocity"));
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
