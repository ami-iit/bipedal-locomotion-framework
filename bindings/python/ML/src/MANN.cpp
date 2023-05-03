/**
 * @file MANN.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ML/MANN.h>

#include <BipedalLocomotion/bindings/ML/MANN.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ML
{

void CreateMANN(pybind11::module& module)
{
    namespace py = ::pybind11;
    namespace ML = BipedalLocomotion::ML;
    namespace System = BipedalLocomotion::System;

    py::class_<ML::MANNInput>(module, "MANNInput")
        .def(py::init())
        .def_readwrite("base_position_trajectory", &ML::MANNInput::basePositionTrajectory)
        .def_readwrite("facing_direction_trajectory", &ML::MANNInput::facingDirectionTrajectory)
        .def_readwrite("base_velocity_trajectory", &ML::MANNInput::baseVelocitiesTrajectory)
        .def_readwrite("joint_positions", &ML::MANNInput::jointPositions)
        .def_readwrite("joint_velocities", &ML::MANNInput::jointVelocities);

    py::class_<ML::MANNOutput>(module, "MANNOutput")
        .def(py::init())
        .def_readwrite("future_base_position_trajectory",
                       &ML::MANNOutput::futureBasePositionTrajectory)
        .def_readwrite("future_facing_direction_trajectory",
                       &ML::MANNOutput::futureFacingDirectionTrajectory)
        .def_readwrite("future_base_velocities_trajectory",
                       &ML::MANNOutput::futureBaseVelocitiesTrajectory)
        .def_readwrite("joint_positions", &ML::MANNOutput::jointPositions)
        .def_readwrite("joint_velocities", &ML::MANNOutput::jointVelocities)
        .def_readwrite("projected_base_velocity", &ML::MANNOutput::projectedBaseVelocity);

    BipedalLocomotion::bindings::System::CreateAdvanceable<ML::MANNInput, //
                                                           ML::MANNOutput>(module, "MANN");
    py::class_<ML::MANN, System::Advanceable<ML::MANNInput, ML::MANNOutput>>(module, "MANN")
        .def(py::init());
}

} // namespace ML
} // namespace bindings
} // namespace BipedalLocomotion
