/**
 * @file RosClock.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/cast.h>
#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/System/RosClock.h>
#include <BipedalLocomotion/bindings/System/RosClock.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{

void CreateRosClock(pybind11::module& module)
{
    namespace py = ::pybind11;

    py::class_<::BipedalLocomotion::System::RosClock,
               ::BipedalLocomotion::System::IClock>(module, "RosClock");
}

void CreateRosClockFactory(pybind11::module& module)
{
    namespace py = ::pybind11;

    py::class_<::BipedalLocomotion::System::RosClockFactory,
               ::BipedalLocomotion::System::ClockFactory,
               std::shared_ptr<::BipedalLocomotion::System::RosClockFactory>>(module,
                                                                               "RosClockFactory")
        .def(py::init<const std::vector<std::string>&>(), py::arg("args"));
    ;
}

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
