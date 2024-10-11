/**
 * @file VelMANNAutoregressiveInputBuilder.cpp
 * @authors Evelyn D'Elia
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ML/VelMANNAutoregressiveInputBuilder.h>
#include <BipedalLocomotion/bindings/ML/VelMANNAutoregressiveInputBuilder.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ML
{

void CreateVelMANNAutoregressiveInputBuilder(pybind11::module& module)
{
    namespace py = ::pybind11;
    namespace ML = BipedalLocomotion::ML;
    namespace System = BipedalLocomotion::System;

    py::class_<ML::VelMANNDirectionalInput>(module, "VelMANNDirectionalInput")
        .def(py::init())
        .def_readwrite("motion_direction", &ML::VelMANNDirectionalInput::motionDirection)
        .def_readwrite("base_direction", &ML::VelMANNDirectionalInput::baseDirection);

    BipedalLocomotion::bindings::System::CreateAdvanceable<ML::VelMANNDirectionalInput, //
                                                           ML::VelMANNAutoregressiveInput> //
        (module, "VelMANNAutoregressiveInputBuilder");
    py::class_<ML::VelMANNAutoregressiveInputBuilder,
               System::Advanceable<ML::VelMANNDirectionalInput, //
                                   ML::VelMANNAutoregressiveInput>>(module,
                                                                 "VelMANNAutoregressiveInputBuilder")
        .def(py::init());
}

} // namespace ML
} // namespace bindings
} // namespace BipedalLocomotion
