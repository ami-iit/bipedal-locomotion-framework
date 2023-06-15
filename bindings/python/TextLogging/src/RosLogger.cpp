/**
 * @file RosLoggerFactory.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/cast.h>
#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TextLogging/RosLogger.h>
#include <BipedalLocomotion/bindings/TextLogging/RosLogger.h>

#include <string_view>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TextLogging
{

void CreateRosLoggerFactory(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TextLogging;

    py::class_<RosLoggerFactory, //
               LoggerFactory,
               std::shared_ptr<RosLoggerFactory>>(module,
                                                  "RosLoggerFactory")
        .def(py::init<const std::string_view&>(), py::arg("name") = "blf");
}

} // namespace TextLogging
} // namespace bindings
} // namespace BipedalLocomotion
