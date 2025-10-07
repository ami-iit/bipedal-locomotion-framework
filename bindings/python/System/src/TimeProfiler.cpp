/**
 * @file TimeProfiler.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/System/TimeProfiler.h>
#include <BipedalLocomotion/bindings/System/TimeProfiler.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{

void CreateTimeProfiler(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;

    py::class_<TimeProfiler>(module, "TimeProfiler")
        .def(py::init())
        .def("set_period", &TimeProfiler::setPeriod, py::arg("max_counter"))
        .def("add_timer", &TimeProfiler::addTimer, py::arg("name"))
        .def("set_init_time", &TimeProfiler::setInitTime, py::arg("name"))
        .def("set_end_time", &TimeProfiler::setEndTime, py::arg("name"))
        .def("profiling", &TimeProfiler::profiling);
}

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
