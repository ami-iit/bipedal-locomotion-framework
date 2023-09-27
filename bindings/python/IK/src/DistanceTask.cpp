/**
 * @file DistanceTask.cpp
 * @authors Stefano Dafarra
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/DistanceTask.h>

#include <BipedalLocomotion/bindings/IK/DistanceTask.h>
#include <BipedalLocomotion/bindings/IK/IKLinearTask.h>
#include <BipedalLocomotion/bindings/System/LinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateDistanceTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;

    py::class_<DistanceTask, std::shared_ptr<DistanceTask>, IKLinearTask>(module, "DistanceTask")
        .def(py::init())
        .def("set_desired_distance", &DistanceTask::setDesiredDistance, py::arg("desired_distance"))
        .def("set_set_point", &DistanceTask::setSetPoint, py::arg("desired_distance"))
        .def("get_distance", &DistanceTask::getDistance);
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
