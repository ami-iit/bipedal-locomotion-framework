/**
 * @file TextLogging.cpp
 * @authors Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include "BipedalLocomotion/bindings/TextLogging/TextLogging.h"
#include "BipedalLocomotion/TextLogging/Logger.h"

namespace BipedalLocomotion::bindings::TextLogging
{
void CreateTextLogging(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TextLogging;

    py::enum_<Verbosity>(module, "Verbosity", py::arithmetic())
        .value("Trace", Verbosity::Trace)
        .value("Debug", Verbosity::Debug)
        .value("Info", Verbosity::Info)
        .value("Warn", Verbosity::Warn)
        .value("Err", Verbosity::Err)
        .value("Critical", Verbosity::Critical)
        .value("Off", Verbosity::Off);

    module.def("set_verbosity", &setVerbosity, py::arg("verbosity"));
}
} // namespace BipedalLocomotion::bindings::TextLogging
