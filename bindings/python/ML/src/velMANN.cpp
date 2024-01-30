/**
 * @file velMANN.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ML/velMANN.h>

#include <BipedalLocomotion/bindings/ML/velMANN.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ML
{

void CreateVelMANN(pybind11::module& module)
{
    namespace py = ::pybind11;
    namespace ML = BipedalLocomotion::ML;
    namespace System = BipedalLocomotion::System;

    py::class_<ML::velMANNInput>(module, "velMANNInput")
        .def(py::init())
        .def_readwrite("base_linear_velocity_trajectory", &ML::velMANNInput::baseLinearVelocityTrajectory)
        .def_readwrite("base_angular_velocity_trajectory", &ML::velMANNInput::baseAngularVelocityTrajectory)
        .def_readwrite("joint_positions", &ML::velMANNInput::jointPositions)
        .def_readwrite("joint_velocities", &ML::velMANNInput::jointVelocities);

    py::class_<ML::velMANNOutput>(module, "velMANNOutput")
        .def(py::init())
        .def_readwrite("future_base_linear_velocity_trajectory",
                       &ML::velMANNOutput::futureBaseLinearVelocityTrajectory)
        .def_readwrite("future_base_angular_velocity_trajectory",
                       &ML::velMANNOutput::futureBaseAngularVelocityTrajectory)
        .def_readwrite("joint_positions", &ML::velMANNOutput::jointPositions)
        .def_readwrite("joint_velocities", &ML::velMANNOutput::jointVelocities);

    BipedalLocomotion::bindings::System::CreateAdvanceable<ML::velMANNInput, //
                                                           ML::velMANNOutput>(module, "velMANN");
    py::class_<ML::velMANN, System::Advanceable<ML::velMANNInput, ML::velMANNOutput>>(module, "velMANN")
        .def(py::init());
}

} // namespace ML
} // namespace bindings
} // namespace BipedalLocomotion
