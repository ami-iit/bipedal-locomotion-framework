/**
 * @file IClock.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/cast.h>
#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>

#include <BipedalLocomotion/System/IClock.h>
#include <BipedalLocomotion/bindings/System/IClock.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{

void CreateIClock(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;

    py::class_<IClock, //
               IClockTrampoline<>>(module, "IClock")
        .def("now", &IClock::now)
        .def("sleep_for", &IClock::sleepFor, py::arg("sleep_duration"))
        .def("sleep_until", &IClock::sleepUntil, py::arg("time"))
        .def("yield", &IClock::yield);
}

void CreateClockFactory(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;

    py::class_<ClockFactory, //
               ClockFactoryTrampoline<>, std::shared_ptr<ClockFactory>>(module, "ClockFactory");
}


} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
