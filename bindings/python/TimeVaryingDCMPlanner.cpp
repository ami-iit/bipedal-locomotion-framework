/**
 * @file TimeVaryingDCMPlanner.cpp
 * @authors DiegoFerigo
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "BipedalLocomotion/Planners/DCMPlanner.h"
#include "BipedalLocomotion/Planners/TimeVaryingDCMPlanner.h"
#include "BipedalLocomotion/System/Advanceable.h"
#include "bipedal_locomotion_framework.h"

void BipedalLocomotion::bindings::CreateTimeVaryingDCMPlanner(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Planners;

    py::class_<TimeVaryingDCMPlanner, DCMPlanner>(module, "TimeVaryingDCMPlanner")
        .def(py::init())
        .def("initialize", &TimeVaryingDCMPlanner::initialize, py::arg("handler"))
        .def("compute_trajectory", &TimeVaryingDCMPlanner::computeTrajectory)
        .def("get", &TimeVaryingDCMPlanner::get)
        .def("is_valid", &TimeVaryingDCMPlanner::isValid)
        .def("advance", &TimeVaryingDCMPlanner::advance);
}
