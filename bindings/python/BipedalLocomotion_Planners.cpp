/**
 * @file BipedalLocomotion_Planners.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <BipedalLocomotion/Planners/QuinticSpline.h>

#include "BipedalLocomotion_Planners.h"

using namespace BipedalLocomotion::Planners;

namespace BipedalLocomotion
{
namespace bindings
{
namespace
{

namespace py = ::pybind11;
void createQuinticSpline(pybind11::module& module)
{
    py::class_<QuinticSpline>(module, "QuinticSpline")
        .def(py::init())
        .def("set_knots", &QuinticSpline::setKnots)
        .def("set_initial_conditions", &QuinticSpline::setInitialConditions)
        .def("set_final_conditions", &QuinticSpline::setFinalConditions)
        .def("evaluate_point",
             static_cast<bool (QuinticSpline::*)(const double&,
                                                 Eigen::Ref<Eigen::VectorXd>,
                                                 Eigen::Ref<Eigen::VectorXd>,
                                                 Eigen::Ref<Eigen::VectorXd>)>(
                 &QuinticSpline::evaluatePoint));
}

} // namespace

void BipedalLocomotionPlannersBindings(pybind11::module& module)
{
    createQuinticSpline(module);
}

} // namespace bindings
} // namespace BipedalLocomotion
