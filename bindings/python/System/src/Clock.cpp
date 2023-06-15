/**
 * @file Clock.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/cast.h>
#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/bindings/System/IClock.h>

namespace BipedalLocomotion
{
namespace bindings
{

void CreateClock(pybind11::module& module)
{
    namespace py = ::pybind11;

    // Reference an existing object, but do not take ownership.
    // https://pybind11.readthedocs.io/en/stable/advanced/functions.html
    module.def("clock", &::BipedalLocomotion::clock, py::return_value_policy::reference);
}

namespace System
{

void CreateClockBuilder(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;

    py::class_<ClockBuilder>(module, "ClockBuilder")
        .def(py::init<>())
        .def_static("set_factory", &ClockBuilder::setFactory, py::arg("factory"));
}

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
