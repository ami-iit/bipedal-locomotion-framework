/**
 * @file ParametersHandler.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/bindings/System/VariablesHandler.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{

void CreateVariablesHandler(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;

    py::class_<VariablesHandler> variablesHandler(module, "VariablesHandler");

    py::class_<VariablesHandler::VariableDescription>(module, "VariableDescription")
        .def(py::init())
        .def_readonly("offset", &VariablesHandler::VariableDescription::offset)
        .def_readonly("size", &VariablesHandler::VariableDescription::size)
        .def_readonly("name", &VariablesHandler::VariableDescription::name)
        .def("is_valid", &VariablesHandler::VariableDescription::isValid)
        .def_static("invalid_variable", &VariablesHandler::VariableDescription::InvalidVariable);

    variablesHandler.def(py::init())
        .def("add_variable",
             py::overload_cast<const std::string&, const std::size_t&>(
                 &VariablesHandler::addVariable),
             py::arg("name"),
             py::arg("size"))
        .def("add_variable",
             py::overload_cast<const std::string&, const std::vector<std::string>&>(
                 &VariablesHandler::addVariable),
             py::arg("name"),
             py::arg("elements_name"))
        .def("add_variable",
             py::overload_cast<const std::string&, const std::size_t&, const std::vector<std::string>& >(
                 &VariablesHandler::addVariable),
             py::arg("name"),
             py::arg("size"),
             py::arg("elements_name"))
        .def("get_variable",
             py::overload_cast<const std::string&>(&VariablesHandler::getVariable, py::const_),
             py::arg("name"))
        .def("get_number_of_variables", &VariablesHandler::getNumberOfVariables)
        .def("__str__", &VariablesHandler::toString);
}

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
