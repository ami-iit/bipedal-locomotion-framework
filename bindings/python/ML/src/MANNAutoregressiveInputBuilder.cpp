/**
 * @file CreateMANNAutoregressiveInputBuilder.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <iDynTree/Model.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ML/MANNAutoregressiveInputBuilder.h>
#include <BipedalLocomotion/bindings/ML/MANNAutoregressiveInputBuilder.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>
#include <BipedalLocomotion/bindings/type_caster/swig.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ML
{

void CreateMANNAutoregressiveInputBuilder(pybind11::module& module)
{
    namespace py = ::pybind11;
    namespace ML = BipedalLocomotion::ML;
    namespace System = BipedalLocomotion::System;

    py::class_<ML::MANNDirectionalInput>(module, "MANNDirectionalInput")
        .def(py::init())
        .def_readwrite("motion_direction", &ML::MANNDirectionalInput::motionDirection)
        .def_readwrite("facing_direction", &ML::MANNDirectionalInput::facingDirection);

    BipedalLocomotion::bindings::System::CreateAdvanceable<ML::MANNDirectionalInput, //
                                                           ML::MANNAutoregressiveInput> //
        (module, "MANNAutoregressiveInputBuilder");
    py::class_<ML::MANNAutoregressiveInputBuilder,
               System::Advanceable<ML::MANNDirectionalInput, //
                                   ML::MANNAutoregressiveInput>>(module,
                                                                 "MANNAutoregressiveInputBuilder")
        .def(py::init());
}

} // namespace ML
} // namespace bindings
} // namespace BipedalLocomotion
