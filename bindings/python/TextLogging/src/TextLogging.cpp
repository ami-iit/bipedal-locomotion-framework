/**
 * @file TextLogging.cpp
 * @authors Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/TextLogging/LoggerBuilder.h>

#include <BipedalLocomotion/bindings/TextLogging/TextLogging.h>

#include <pybind11/cast.h>
#include <pybind11/pybind11.h>

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

    py::class_<Logger, std::shared_ptr<Logger>>(module, "Logger")
        .def("info", [](Logger& logger, const std::string& msg) { logger.info(msg); })
        .def("warn", [](Logger& logger, const std::string& msg) { logger.warn(msg); })
        .def("error", [](Logger& logger, const std::string& msg) { logger.error(msg); })
        .def("critical", [](Logger& logger, const std::string& msg) { logger.critical(msg); })
        .def("trace", [](Logger& logger, const std::string& msg) { logger.trace(msg); })
        .def("debug", [](Logger& logger, const std::string& msg) { logger.debug(msg); });

    module.def("set_verbosity", &setVerbosity, py::arg("verbosity"));

    py::class_<LoggerFactory, std::shared_ptr<LoggerFactory>>(module, "LoggerFactory");
    py::class_<LoggerBuilder>(module, "LoggerBuilder")
        .def_static("set_factory", &LoggerBuilder::setFactory, py::arg("factory"));

}
} // namespace BipedalLocomotion::bindings::TextLogging

namespace BipedalLocomotion::bindings
{
void CreateLogger(pybind11::module& module)
{
    namespace py = ::pybind11;

    module.def("log", &::BipedalLocomotion::log);
}
} // namespace BipedalLocomotion::bindings
