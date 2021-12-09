/**
 * @file TimeVaryingDCMPlanner.cpp
 * @authors Diego Ferigo, Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/Planners/DCMPlanner.h>
#include <BipedalLocomotion/Planners/TimeVaryingDCMPlanner.h>

#include <BipedalLocomotion/bindings/Planners/TimeVaryingDCMPlanner.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Planners
{

void CreateTimeVaryingDCMPlanner(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Planners;

    py::class_<TimeVaryingDCMPlanner, DCMPlanner>(module, "TimeVaryingDCMPlanner")
        .def(py::init())
        .def("compute_trajectory", &TimeVaryingDCMPlanner::computeTrajectory)
        .def("set_contact_phase_list",
             &TimeVaryingDCMPlanner::setContactPhaseList,
             py::arg("contact_phase_list"))
        .def("set_dcm_reference",
             &TimeVaryingDCMPlanner::setDCMReference,
             py::arg("dcm_reference_matrix"))
        .def("set_initial_state", &TimeVaryingDCMPlanner::setInitialState, py::arg("state"));
}
}
}
}
