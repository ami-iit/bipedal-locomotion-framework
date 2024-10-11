/**
 * @file VelMANN.cpp
 * @authors Evelyn D'Elia
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ML/VelMANN.h>
#include <BipedalLocomotion/bindings/ML/VelMANN.h>
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

    py::class_<ML::VelMANNInput>(module, "VelMANNInput")
        .def(py::init())
        .def_readwrite("base_linear_velocity_trajectory", &ML::VelMANNInput::baseLinearVelocityTrajectory)
        .def_readwrite("base_angular_velocity_trajectory", &ML::VelMANNInput::baseAngularVelocityTrajectory)
        .def_readwrite("joint_positions", &ML::VelMANNInput::jointPositions)
        .def_readwrite("joint_velocities", &ML::VelMANNInput::jointVelocities);

    py::class_<ML::VelMANNOutput>(module, "VelMANNOutput")
        .def(py::init())
        .def_readwrite("future_base_linear_velocity_trajectory",
                       &ML::VelMANNOutput::futureBaseLinearVelocityTrajectory)
        .def_readwrite("future_base_angular_velocity_trajectory",
                       &ML::VelMANNOutput::futureBaseAngularVelocityTrajectory)
        .def_readwrite("joint_positions", &ML::VelMANNOutput::jointPositions)
        .def_readwrite("joint_velocities", &ML::VelMANNOutput::jointVelocities);

    BipedalLocomotion::bindings::System::CreateAdvanceable<ML::VelMANNInput, //
                                                           ML::VelMANNOutput>(module, "VelMANN");
    py::class_<ML::VelMANN, System::Advanceable<ML::VelMANNInput, ML::VelMANNOutput>>(module, "VelMANN")
        .def(py::init());
}

} // namespace ML
} // namespace bindings
} // namespace BipedalLocomotion
