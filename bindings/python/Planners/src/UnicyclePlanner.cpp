/**
 * @file UnicyclePlanner.cpp
 * @authors Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include "BipedalLocomotion/Planners/UnicyclePlanner.h"
#include "BipedalLocomotion/Contacts/ContactList.h"
#include "BipedalLocomotion/ParametersHandler/IParametersHandler.h"
#include "BipedalLocomotion/System/Advanceable.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace BipedalLocomotion::bindings::Planners
{

void CreateUnicyclePlanner(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Planners;
    using namespace BipedalLocomotion::System;
    using namespace BipedalLocomotion::ParametersHandler;

    py::class_<UnicycleKnot>(module, "UnicycleKnot")
        .def(py::init<const Eigen::Vector2d&, const Eigen::Vector2d&, double>(),
             py::arg("position") = std::make_tuple(0.0, 0.0),
             py::arg("velocity") = std::make_tuple(0.0, 0.0),
             py::arg("time") = 0.0)
        .def_readwrite("time", &UnicycleKnot::time)
        .def_readwrite("x", &UnicycleKnot::x)
        .def_readwrite("y", &UnicycleKnot::y)
        .def_readwrite("dx", &UnicycleKnot::dx)
        .def_readwrite("dy", &UnicycleKnot::dy)
        .def("__eq__", &UnicycleKnot::operator==, py::is_operator());

    py::class_<UnicyclePlannerInput>(module, "UnicyclePlannerInput")
        .def(py::init<const std::vector<UnicycleKnot>&, const double, const double>(),
             py::arg("knots") = std::vector<UnicycleKnot>{},
             py::arg("tf") = 0.0,
             py::arg("t0") = 0.0)
        .def_readwrite("t0", &UnicyclePlannerInput::t0)
        .def_readwrite("tf", &UnicyclePlannerInput::tf)
        .def_readwrite("knots", &UnicyclePlannerInput::knots);

    py::class_<UnicyclePlannerOutput>(module, "UnicyclePlannerOutput")
        .def(py::init<const Contacts::ContactList&, const Contacts::ContactList&>(),
             py::arg("left") = Contacts::ContactList(),
             py::arg("right") = Contacts::ContactList())
        .def_readwrite("left", &UnicyclePlannerOutput::left)
        .def_readwrite("right", &UnicyclePlannerOutput::right);

    py::class_<Advanceable<UnicyclePlannerInput, UnicyclePlannerOutput>>( //
        module,
        "UnicyclePlannerAdvanceable");

    py::class_<UnicyclePlanner, Advanceable<UnicyclePlannerInput, UnicyclePlannerOutput>>( //
        module,
        "UnicyclePlanner")
        .def(py::init())
        .def(
            "initialize",
            [](UnicyclePlanner& impl, std::shared_ptr<const IParametersHandler> handler) -> bool {
                return impl.initialize(handler);
            },
            py::arg("handler"))
        .def("get_output", &UnicyclePlanner::getOutput)
        .def("is_output_valid", &UnicyclePlanner::isOutputValid)
        .def("set_input", &UnicyclePlanner::setInput, py::arg("input"))
        .def("advance", &UnicyclePlanner::advance);
}

} // namespace BipedalLocomotion::bindings::Planners
