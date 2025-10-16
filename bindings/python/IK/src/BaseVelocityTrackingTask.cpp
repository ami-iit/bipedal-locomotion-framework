/**
 * @file BaseVelocityTrackingTask.cpp
 * @authors Davide Gorbani
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/BaseVelocityTrackingTask.h>
#include <BipedalLocomotion/IK/IKLinearTask.h>

#include <BipedalLocomotion/bindings/IK/BaseVelocityTrackingTask.h>
#include <BipedalLocomotion/bindings/System/LinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateBaseVelocityTrackingTask(pybind11::module& module)
{
    namespace py = pybind11;
    using namespace BipedalLocomotion::IK;
    using namespace BipedalLocomotion::System;

    py::class_<BaseVelocityTrackingTask,
               std::shared_ptr<BaseVelocityTrackingTask>,
               IKLinearTask>(module, "BaseVelocityTrackingTask")
        .def(py::init<>())
        .def("set_set_point",
             &BaseVelocityTrackingTask::setSetPoint,
             py::arg("desired_base_velocity"));
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
