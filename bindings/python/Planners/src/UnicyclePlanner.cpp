/**
 * @file UnicyclePlanner.cpp
 * @authors Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Planners/UnicyclePlanner.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <BipedalLocomotion/bindings/System/Advanceable.h>

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
        .def("__eq__", &UnicycleKnot::operator==, py::is_operator())
        .def("__repr__", [](const UnicycleKnot& k) {
            return std::string("UnicycleKnot(") + //
                   "x=" + std::to_string(k.x) + ", " + //
                   "y=" + std::to_string(k.y) + ", " + //
                   "dx=" + std::to_string(k.dx) + ", " + //
                   "dy=" + std::to_string(k.dy) + ", " + //
                   "time=" + std::to_string(k.time) + //
                   ")";
        });

    py::class_<UnicyclePlannerInput>(module, "UnicyclePlannerInput")
        .def(py::init<const std::vector<UnicycleKnot>&,
                      const double,
                      const std::optional<Contacts::PlannedContact>&,
                      const std::optional<Contacts::PlannedContact>&,
                      const double>(),
             py::arg("knots") = std::vector<UnicycleKnot>{},
             py::arg("tf") = 0.0,
             py::arg("initial_left_contact") = std::nullopt,
             py::arg("initial_right_contact") = std::nullopt,
             py::arg("t0") = 0.0)
        .def_readwrite("t0", &UnicyclePlannerInput::t0)
        .def_readwrite("tf", &UnicyclePlannerInput::tf)
        .def_readwrite("initial_left_contact", &UnicyclePlannerInput::initialLeftContact)
        .def_readwrite("initial_right_contact", &UnicyclePlannerInput::initialRightContact)
        .def_readwrite("knots", &UnicyclePlannerInput::knots)
        .def("__repr__", [](const UnicyclePlannerInput& i) {
            auto printContact
                = [](const std::optional<Contacts::PlannedContact>& c) -> std::string {
                return c ? "PlannedContact(" + c->name + ")" : "None";
            };
            return std::string("UnicyclePlannerInput(") + //
                   "t0=" + std::to_string(i.t0) + ", " + //
                   "tf=" + std::to_string(i.tf) + ", " + //
                   "knots=KnotList(" + std::to_string(i.knots.size()) + "), " + //
                   "initial_left_contact=" + printContact(i.initialLeftContact) + ", " + //
                   "initial_right_contact=" + printContact(i.initialRightContact) + //
                   ")";
        });

    py::class_<UnicyclePlannerOutput>(module, "UnicyclePlannerOutput")
        .def(py::init<const Contacts::ContactList&, const Contacts::ContactList&>(),
             py::arg("left") = Contacts::ContactList(),
             py::arg("right") = Contacts::ContactList())
        .def_readwrite("left", &UnicyclePlannerOutput::left)
        .def_readwrite("right", &UnicyclePlannerOutput::right)
        .def("__repr__", [](const UnicyclePlannerOutput& o) {
            auto printContactList = [](const Contacts::ContactList& l) -> std::string {
                return "ContactList(" + std::to_string(l.size()) + ")";
            };
            return std::string("UnicyclePlannerOutput(") + //
                   "left=" + printContactList(o.left) + ", " + //
                   "right=" + printContactList(o.right) + //
                   ")";
        });

    BipedalLocomotion::bindings::System::CreateAdvanceable<UnicyclePlannerInput,
                                                           UnicyclePlannerOutput>( //
        module,
        "UnicyclePlanner");

    py::class_<UnicyclePlanner, Advanceable<UnicyclePlannerInput, UnicyclePlannerOutput>>( //
        module,
        "UnicyclePlanner")
        .def(py::init());
}

} // namespace BipedalLocomotion::bindings::Planners
