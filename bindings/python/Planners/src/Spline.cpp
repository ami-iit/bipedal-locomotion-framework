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

#include <BipedalLocomotion/Planners/CubicSpline.h>
#include <BipedalLocomotion/Planners/QuinticSpline.h>
#include <BipedalLocomotion/Planners/Spline.h>

#include <BipedalLocomotion/bindings/Planners/Spline.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Planners
{

struct PySplineState : public BipedalLocomotion::Math::TrajectoryPoint<Eigen::VectorXd>
{
    using BipedalLocomotion::Math::TrajectoryPoint<Eigen::VectorXd>::TrajectoryPoint;
};

struct PySpline : public BipedalLocomotion::Planners::Spline
{
    using BipedalLocomotion::Planners::Spline::Spline;
};

struct PyCubicSpline : public BipedalLocomotion::Planners::CubicSpline
{
    using BipedalLocomotion::Planners::CubicSpline::CubicSpline;
};

struct PyQuinticSpline : public BipedalLocomotion::Planners::QuinticSpline
{
    using BipedalLocomotion::Planners::QuinticSpline::QuinticSpline;
};

void CreateSpline(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Math;

    py::class_<PySplineState, TrajectoryPoint<Eigen::VectorXd>>(module, "SplineState")
        .def(py::init([]() {
            PyErr_WarnEx(PyExc_DeprecationWarning,
                         "bipedal_locomotion.planners.SplineState is deprecated please use "
                         "bipedal_locomotion.math.TrajectoryPoint instead.",
                         1);
            return PySplineState();
        }));

    py::class_<PySpline, //
               BipedalLocomotion::Planners::Spline>(module, "Spline");
}

void CreateCubicSpline(pybind11::module& module)
{
    namespace py = ::pybind11;
    py::class_<PyCubicSpline, //
               BipedalLocomotion::Planners::CubicSpline>(module, "CubicSpline")
        .def(py::init([]() {
            PyErr_WarnEx(PyExc_DeprecationWarning,
                         "bipedal_locomotion.planners.CubicSpline is deprecated, use "
                         "bipedal_locomotion.math.CubicSpline instead.",
                         1);
            return std::make_unique<PyCubicSpline>();
        }))
        .def("set_final_conditions",
             py::overload_cast<Eigen::Ref<const Eigen::VectorXd>>(
                 &BipedalLocomotion::Planners::CubicSpline::setFinalConditions),
             py::arg("velocity"))
        .def("set_initial_conditions",
             py::overload_cast<Eigen::Ref<const Eigen::VectorXd>>(
                 &BipedalLocomotion::Planners::CubicSpline::setInitialConditions),
             py::arg("velocity"));
}

void CreateQuinticSpline(pybind11::module& module)
{
    namespace py = ::pybind11;
    py::class_<PyQuinticSpline, //
               BipedalLocomotion::Planners::QuinticSpline>(module, "QuinticSpline")
        .def(py::init([]() {
            PyErr_WarnEx(PyExc_DeprecationWarning,
                         "bipedal_locomotion.planners.QuinticSpline is deprecated, use "
                         "bipedal_locomotion.math.QuinticSpline instead.",
                         1);
            return std::make_unique<PyQuinticSpline>();
        }));
}

} // namespace Planners
} // namespace bindings
} // namespace BipedalLocomotion
