/**
 * @file Spline.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <Eigen/Dense>

#include <chrono>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>

#include <BipedalLocomotion/Planners/QuinticSpline.h>
#include <BipedalLocomotion/Planners/CubicSpline.h>
#include <BipedalLocomotion/Planners/Spline.h>

#include <BipedalLocomotion/bindings/Planners/Spline.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Planners
{

void CreateSpline(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Planners;

    py::class_<SplineState>(module, "SplineState")
        .def(py::init())
        .def_readwrite("position", &SplineState::position)
        .def_readwrite("velocity", &SplineState::velocity)
        .def_readwrite("acceleration", &SplineState::acceleration);

    py::class_<Spline>(module, "Spline")
        .def("set_advance_time_step", &Spline::setAdvanceTimeStep, py::arg("dt"))
        .def("set_knots", &Spline::setKnots)
        .def("set_initial_conditions",
             &Spline::setInitialConditions,
             py::arg("velocity"),
             py::arg("acceleration"))
        .def("set_final_conditions",
             &Spline::setFinalConditions,
             py::arg("velocity"),
             py::arg("acceleration"))
        .def("evaluate_point",
             py::overload_cast<const std::chrono::nanoseconds&,
                               Eigen::Ref<Eigen::VectorXd>,
                               Eigen::Ref<Eigen::VectorXd>,
                               Eigen::Ref<Eigen::VectorXd>>(&Spline::evaluatePoint),
             py::arg("time"),
             py::arg("position"),
             py::arg("velocity"),
             py::arg("acceleration"))
        .def("get_output", &Spline::getOutput)
        .def("is_output_valid", &Spline::isOutputValid)
        .def("advance", &Spline::advance);
}

void CreateCubicSpline(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Planners;

    py::class_<CubicSpline, Spline>(module, "CubicSpline")
        .def(py::init());
}

void CreateQuinticSpline(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Planners;

    py::class_<QuinticSpline, Spline>(module, "QuinticSpline")
        .def(py::init());
}

} // namespace Planners
} // namespace bindings
} // namespace BipedalLocomotion
