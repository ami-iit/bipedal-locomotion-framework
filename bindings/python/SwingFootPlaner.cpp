/**
 * @file SwingFootPlanner.cpp
 * @authors Diego Ferigo
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "BipedalLocomotion/Planners/SwingFootPlanner.h"
#include "BipedalLocomotion/System/Advanceable.h"
#include "bipedal_locomotion.h"

void BipedalLocomotion::bindings::CreateSwingFootPlanner(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Planners;
    using namespace BipedalLocomotion::System;

    py::class_<SwingFootPlannerState>(module, "SwingFootPlannerState")
        .def(py::init())
        .def_readwrite("is_in_contact", &SwingFootPlannerState::isInContact)
        .def_property(
            "transform",
            [](const SwingFootPlannerState& s) { return s.transform.coeffs(); },
            [](SwingFootPlannerState& s, const Eigen::Ref<Eigen::VectorXd>& coeffs) {
                s.transform.coeffs() = coeffs;
            })
        .def_property(
            "mixed_velocity",
            [](const SwingFootPlannerState& s) { return s.mixedVelocity.coeffs(); },
            [](SwingFootPlannerState& s, const Eigen::Ref<Eigen::VectorXd>& coeffs) {
                s.mixedVelocity.coeffs() = coeffs;
            })
        .def_property(
            "mixed_acceleration",
            [](const SwingFootPlannerState& s) { return s.mixedAcceleration.coeffs(); },
            [](SwingFootPlannerState& s, const Eigen::Ref<Eigen::VectorXd>& coeffs) {
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
