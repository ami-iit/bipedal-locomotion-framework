/**
 * @file Spline.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <Eigen/Dense>

#include <chrono>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/Math/CubicSpline.h>
#include <BipedalLocomotion/Math/LinearSpline.h>
#include <BipedalLocomotion/Math/QuinticSpline.h>
#include <BipedalLocomotion/Math/Spline.h>
#include <BipedalLocomotion/Math/ZeroOrderSpline.h>

#include <BipedalLocomotion/bindings/Math/Spline.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Math
{

template <typename T> void CreateTrajectoryPoint(pybind11::module& module, const std::string& name)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Math;

    py::class_<TrajectoryPoint<T>>(module, name.c_str())
        .def(py::init())
        .def_readwrite("position", &TrajectoryPoint<T>::position)
        .def_readwrite("velocity", &TrajectoryPoint<T>::velocity)
        .def_readwrite("acceleration", &TrajectoryPoint<T>::acceleration);
}

template <typename T> void CreateSplineTmp(pybind11::module& module, const std::string& name)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Math;

    BipedalLocomotion::bindings::System::CreateSource<TrajectoryPoint<T>>(module, name);
    py::class_<Spline<T>, //
               BipedalLocomotion::System::Source<TrajectoryPoint<T>>>(module, name.c_str())
        .def("set_advance_time_step", &Spline<T>::setAdvanceTimeStep, py::arg("dt"))
        .def("set_knots", &Spline<T>::setKnots)
        .def("set_initial_conditions",
             py::overload_cast<Eigen::Ref<const T>, Eigen::Ref<const T>>(
                 &Spline<T>::setInitialConditions),
             py::arg("velocity"),
             py::arg("acceleration"))
        .def("set_final_conditions",
             py::overload_cast<Eigen::Ref<const T>, Eigen::Ref<const T>>(
                 &Spline<T>::setFinalConditions),
             py::arg("velocity"),
             py::arg("acceleration"))
        .def("evaluate_point",
             py::overload_cast<const std::chrono::nanoseconds&,
                               Eigen::Ref<T>,
                               Eigen::Ref<T>,
                               Eigen::Ref<T>>(&Spline<T>::evaluatePoint),
             py::arg("time"),
             py::arg("position"),
             py::arg("velocity"),
             py::arg("acceleration"))
        .def("evaluate_ordered_points",
             py::overload_cast<const std::vector<std::chrono::nanoseconds>&,
                               std::vector<T>&,
                               std::vector<T>&,
                               std::vector<T>&>(&Spline<T>::evaluateOrderedPoints),
             py::arg("time"),
             py::arg("position"),
             py::arg("velocity"),
             py::arg("acceleration"));
}

template <typename T>
void CreateZeroOrderSplineTmp(pybind11::module& module, const std::string& name)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Math;
    py::class_<ZeroOrderSpline<T>, Spline<T>>(module, name.c_str()).def(py::init());
}

template <typename T> void CreateLinearSplineTmp(pybind11::module& module, const std::string& name)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Math;
    py::class_<LinearSpline<T>, Spline<T>>(module, name.c_str()).def(py::init());
}

template <typename T> void CreateCubicSplineTmp(pybind11::module& module, const std::string& name)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Math;
    py::class_<CubicSpline<T>, Spline<T>>(module, name.c_str()).def(py::init());
}

template <typename T> void CreateQuinticSplineTmp(pybind11::module& module, const std::string& name)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Math;
    py::class_<QuinticSpline<T>, Spline<T>>(module, name.c_str()).def(py::init());
}

void CreateSpline(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Math;

    CreateTrajectoryPoint<Eigen::VectorXd>(module, "TrajectoryPoint");
    CreateSplineTmp<Eigen::VectorXd>(module, "Spline");
}

void CreateZeroOrderSpline(pybind11::module& module)
{
    CreateZeroOrderSplineTmp<Eigen::VectorXd>(module, "ZeroOrderSpline");
}

void CreateLinearSpline(pybind11::module& module)
{
    CreateLinearSplineTmp<Eigen::VectorXd>(module, "LinearSpline");
}

void CreateCubicSpline(pybind11::module& module)
{
    CreateCubicSplineTmp<Eigen::VectorXd>(module, "CubicSpline");
}

void CreateQuinticSpline(pybind11::module& module)
{
    CreateQuinticSplineTmp<Eigen::VectorXd>(module, "QuinticSpline");
}

} // namespace Math
} // namespace bindings
} // namespace BipedalLocomotion
