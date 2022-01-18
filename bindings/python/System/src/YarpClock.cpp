/**
 * @file YarpClock.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/cast.h>
#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/bindings/System/YarpClock.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{

void CreateYarpClock(pybind11::module& module)
{
    namespace py = ::pybind11;

    py::class_<::BipedalLocomotion::System::YarpClock,
               ::BipedalLocomotion::System::IClock>(module, "YarpClock");
}

void CreateYarpClockFactory(pybind11::module& module)
{
    namespace py = ::pybind11;

    py::class_<::BipedalLocomotion::System::YarpClockFactory,
               ::BipedalLocomotion::System::ClockFactory,
               std::shared_ptr<::BipedalLocomotion::System::YarpClockFactory>>(module,
                                                                               "YarpClockFactory")
        .def(py::init<>());
    ;
}

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
