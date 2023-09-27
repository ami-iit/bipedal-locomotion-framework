/**
 * @file GravityTask.cpp
 * @authors Stefano Dafarra
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/GravityTask.h>

#include <BipedalLocomotion/bindings/IK/GravityTask.h>
#include <BipedalLocomotion/bindings/IK/IKLinearTask.h>
#include <BipedalLocomotion/bindings/System/LinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateGravityTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;

    py::class_<GravityTask, std::shared_ptr<GravityTask>, IKLinearTask>(module, "GravityTask")
        .def(py::init())
        .def("set_desired_gravity_direction_in_target_frame",
             &GravityTask::setDesiredGravityDirectionInTargetFrame,
             py::arg("desired_gravity_direction"))
        .def("set_feedforward_velocity_in_target_frame",
             &GravityTask::setFeedForwardVelocityInTargetFrame,
             py::arg("feedforward_velocity"))
        .def("set_set_point",
             &GravityTask::setSetPoint,
             py::arg("desired_gravity_direction"),
             py::arg("feedforward_velocity") = Eigen::Vector3d::Zero());
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
