/**
 * @file SwingFootPlanner.cpp
 * @authors Diego Ferigo, Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/Planners/SwingFootPlanner.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <BipedalLocomotion/bindings/Planners/SwingFootPlanner.h>

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

    py::class_<Advanceable<SwingFootPlannerState>>(module, "SwingFootPlannerStateAdvanceable");

    py::class_<SwingFootPlanner, Advanceable<SwingFootPlannerState>>(module, "SwingFootPlanner")
        .def(py::init())
        .def("initialize", &SwingFootPlanner::initialize, py::arg("handler"))
        .def("set_contact_list", &SwingFootPlanner::setContactList, py::arg("contact_list"))
        .def("get", &SwingFootPlanner::get)
        .def("is_valid", &SwingFootPlanner::isValid)
        .def("advance", &SwingFootPlanner::advance);
}

} // namespace Planners
} // namespace bindings
} // namespace BipedalLocomotion
