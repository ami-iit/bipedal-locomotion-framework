/**
 * @file BipedalLocomotion_GenericContainer.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include "BipedalLocomotion_ParametersHandler.h"
using namespace BipedalLocomotion::ParametersHandler;

namespace BipedalLocomotion
{
namespace bindings
{
namespace
{

namespace py = ::pybind11;
void createIParameterHandler(pybind11::module& module)
{
    py::class_<IParametersHandler>(module, "IParameter_handler");
}

void createStdParameterHandler(pybind11::module& module)
{
    py::class_<StdImplementation, IParametersHandler>(module, "StdParameterHandler")
        .def("set_parameter",
             static_cast<void (StdImplementation::*)(const std::string&, const int&)>(
                 &StdImplementation::setParameter))
        .def("set_parameter",
             static_cast<void (StdImplementation::*)(const std::string&, const double&)>(
                 &StdImplementation::setParameter))
        .def("set_parameter",
             static_cast<void (StdImplementation::*)(const std::string&, const std::string&)>(
                 &StdImplementation::setParameter))
        .def("set_parameter",
             static_cast<void (StdImplementation::*)(const std::string&, const bool&)>(
                 &StdImplementation::setParameter))
        .def("set_parameter",
             static_cast<void (StdImplementation::*)(const std::string&, const std::vector<bool>&)>(
                 &StdImplementation::setParameter))
        .def("set_parameter",
             static_cast<void (
                 StdImplementation::*)(const std::string&,
                                       BipedalLocomotion::GenericContainer::Vector<const int>::Ref)>(
                 &StdImplementation::setParameter))
        .def("set_parameter",
             static_cast<void (StdImplementation::*)(const std::string&,
                                                     BipedalLocomotion::GenericContainer::Vector<
                                                         const double>::Ref)>(
                 &StdImplementation::setParameter))
        .def("set_parameter",
             static_cast<void (StdImplementation::*)(const std::string&,
                                                     BipedalLocomotion::GenericContainer::Vector<
                                                         const std::string>::Ref)>(
                 &StdImplementation::setParameter))
        .def("get_parameter",
             static_cast<bool (StdImplementation::*)(const std::string&, int&) const>(
                 &StdImplementation::getParameter))
        .def("get_parameter",
             static_cast<bool (StdImplementation::*)(const std::string&, double&) const>(
                 &StdImplementation::getParameter))
        .def("get_parameter",
             static_cast<bool (StdImplementation::*)(const std::string&, std::string&) const>(
                 &StdImplementation::getParameter))
        .def("get_parameter",
             static_cast<bool (StdImplementation::*)(const std::string&, bool&) const>(
                 &StdImplementation::getParameter))
        .def("get_parameter",
             static_cast<bool (StdImplementation::*)(const std::string&, std::vector<bool>&) const>(
                 &StdImplementation::getParameter))
        .def("get_parameter",
             static_cast<bool (
                 StdImplementation::*)(const std::string&,
                                       BipedalLocomotion::GenericContainer::Vector<int>::Ref) const>(
                 &StdImplementation::getParameter))
        .def("get_parameter",
             static_cast<bool (
                 StdImplementation::*)(const std::string&,
                                       BipedalLocomotion::GenericContainer::Vector<double>::Ref)
                             const>(&StdImplementation::getParameter))
        .def("get_parameter",
             static_cast<bool (
                 StdImplementation::*)(const std::string&,
                                       BipedalLocomotion::GenericContainer::Vector<std::string>::Ref)
                             const>(&StdImplementation::getParameter))
        .def("is_empty", &StdImplementation::isEmpty)
        .def("__repr__", &StdImplementation::toString)
        .def("clear", &StdImplementation::clear);
}

} // namespace

void BipedalLocomotionParametersHandlerBindings(pybind11::module& module)
{
    createIParameterHandler(module);
    createStdParameterHandler(module);
}

} // namespace bindings
} // namespace BipedalLocomotion
