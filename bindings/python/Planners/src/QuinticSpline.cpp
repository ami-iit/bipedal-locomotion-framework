/**
 * @file QuinticSpline.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */


#include <Eigen/Dense>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/Planners/QuinticSpline.h>

#include <BipedalLocomotion/bindings/Planners/QuinticSpline.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Planners
{

void CreateQuinticSpline(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Planners;

    py::class_<QuinticSpline>(module, "QuinticSpline")
        .def(py::init())
        .def("set_knots", &QuinticSpline::setKnots)
        .def("set_initial_conditions",
             &QuinticSpline::setInitialConditions,
             py::arg("velocity"),
             py::arg("acceleration"))
        .def("set_final_conditions",
             &QuinticSpline::setFinalConditions,
             py::arg("velocity"),
             py::arg("acceleration"))
        .def("evaluate_point",
             py::overload_cast<const double&,
                               Eigen::Ref<Eigen::VectorXd>,
                               Eigen::Ref<Eigen::VectorXd>,
                               Eigen::Ref<Eigen::VectorXd>>(&QuinticSpline::evaluatePoint),
             py::arg("time"),
             py::arg("position"),
             py::arg("velocity"),
             py::arg("acceleration"));
}

} // namespace Planners
} // namespace bindings
} // namespace BipedalLocomotion
