/**
 * @file velMANNAutoregressiveInputBuilder.cpp
 * @authors Evelyn D'Elia
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <iDynTree/Model/Model.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ML/velMANNAutoregressiveInputBuilder.h>
#include <BipedalLocomotion/bindings/ML/velMANNAutoregressiveInputBuilder.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>
#include <BipedalLocomotion/bindings/type_caster/swig.h>

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

    py::class_<ML::velMANNDirectionalInput>(module, "velMANNDirectionalInput")
        .def(py::init())
        .def_readwrite("motion_direction", &ML::velMANNDirectionalInput::motionDirection)
        .def_readwrite("base_direction", &ML::velMANNDirectionalInput::baseDirection);

    BipedalLocomotion::bindings::System::CreateAdvanceable<ML::velMANNDirectionalInput, //
                                                           ML::velMANNAutoregressiveInput> //
        (module, "velMANNAutoregressiveInputBuilder");
    py::class_<ML::velMANNAutoregressiveInputBuilder,
               System::Advanceable<ML::velMANNDirectionalInput, //
                                   ML::velMANNAutoregressiveInput>>(module,
                                                                 "velMANNAutoregressiveInputBuilder")
        .def(py::init());
}

} // namespace ML
} // namespace bindings
} // namespace BipedalLocomotion
