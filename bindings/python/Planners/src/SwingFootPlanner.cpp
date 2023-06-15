/**
 * @file SwingFootPlanner.cpp
 * @authors Diego Ferigo, Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Planners/SwingFootPlanner.h>

#include <BipedalLocomotion/bindings/Planners/SwingFootPlanner.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Planners
{

void CreateSwingFootPlanner(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Planners;
    using namespace BipedalLocomotion::System;
    using namespace BipedalLocomotion::ParametersHandler;

    py::class_<SwingFootPlannerState>(module, "SwingFootPlannerState")
        .def(py::init())
        .def_readwrite("is_in_contact", &SwingFootPlannerState::isInContact)
        .def_readwrite("transform", &SwingFootPlannerState::transform)
        .def_property(
            "mixed_velocity",
            [](const SwingFootPlannerState& s) //
            -> decltype(SwingFootPlannerState::mixedVelocity)::DataType {
                return s.mixedVelocity.coeffs();
            },
            [](SwingFootPlannerState& s,
               decltype(SwingFootPlannerState::mixedVelocity)::DataType& coeffs) {
                s.mixedVelocity.coeffs() = coeffs;
            })
        .def_property(
            "mixed_acceleration",
            [](const SwingFootPlannerState& s) //
            -> decltype(SwingFootPlannerState::mixedAcceleration)::DataType {
                return s.mixedAcceleration.coeffs();
            },
            [](SwingFootPlannerState& s,
               decltype(SwingFootPlannerState::mixedAcceleration)::DataType& coeffs) {
                s.mixedAcceleration.coeffs() = coeffs;
            });

    BipedalLocomotion::bindings::System::CreateSource<SwingFootPlannerState> //
        (module, "SwingFootPlanner");

    py::class_<SwingFootPlanner, Source<SwingFootPlannerState>>(module, "SwingFootPlanner")
        .def(py::init())
        .def("set_contact_list", &SwingFootPlanner::setContactList, py::arg("contact_list"))
        .def("set_time", &SwingFootPlanner::setTime, py::arg("time"));
}

} // namespace Planners
} // namespace bindings
} // namespace BipedalLocomotion
