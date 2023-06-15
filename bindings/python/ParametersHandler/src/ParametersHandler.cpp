/**
 * @file ParametersHandler.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/bindings/ParametersHandler/ParametersHandler.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ParametersHandler
{

void CreateIParameterHandler(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::ParametersHandler;

    py::class_<GenericContainer::Vector<const int>>(module, "GenericContainerVectorInt");
    py::class_<GenericContainer::Vector<const double>>(module, "GenericContainerVectorDouble");

    py::class_<IParametersHandler, std::shared_ptr<IParametersHandler>>(module,
                                                                        "IParametersHandler")
        // Note: the order of the following overloads matters!
        .def("set_parameter_bool",
             py::overload_cast<const std::string&, const bool&>(&IParametersHandler::setParameter),
             py::arg("name"),
             py::arg("value"))
        .def("set_parameter_int",
             py::overload_cast<const std::string&, const int&>(&IParametersHandler::setParameter),
             py::arg("name"),
             py::arg("value"))
        .def("set_parameter_float",
             py::overload_cast<const std::string&, const double&>(
                 &IParametersHandler::setParameter),
             py::arg("name"),
             py::arg("value"))
        .def("set_parameter_datetime",
             py::overload_cast<const std::string&, const std::chrono::nanoseconds&>(
                 &IParametersHandler::setParameter),
             py::arg("name"),
             py::arg("value"))
        .def("set_parameter_string",
             py::overload_cast<const std::string&, const std::string&>(
                 &IParametersHandler::setParameter),
             py::arg("name"),
             py::arg("value"))
        .def("set_parameter_vector_bool",
             py::overload_cast<const std::string&, const std::vector<bool>&>(
                 &IParametersHandler::setParameter),
             py::arg("name"),
             py::arg("value"))
        .def(
            "set_parameter_vector_int",
            [](IParametersHandler& impl, const std::string& name, const std::vector<int>& vec) {
                impl.setParameter(name, vec);
            },
            py::arg("name"),
            py::arg("value"))
        .def(
            "set_parameter_vector_float",
            [](IParametersHandler& impl, const std::string& name, const std::vector<double>& vec) {
                impl.setParameter(name, vec);
            },
            py::arg("name"),
            py::arg("value"))
        .def(
            "set_parameter_vector_string",
            [](IParametersHandler& impl,
               const std::string& name,
               const std::vector<std::string>& vec) { impl.setParameter(name, vec); },
            py::arg("name"),
            py::arg("value"))
        .def(
            "set_parameter_vector_datetime",
            [](IParametersHandler& impl,
               const std::string& name,
               const std::vector<std::chrono::nanoseconds>& vec) { impl.setParameter(name, vec); },
            py::arg("name"),
            py::arg("value"))
        .def("set_group", &IParametersHandler::setGroup, py::arg("name"), py::arg("new_group"))
        .def(
            "get_group",
            [](IParametersHandler& impl, const std::string& name) {
                auto group = impl.getGroup(name).lock();
                if (group == nullptr)
                {
                    throw py::value_error("Failed to find the group named " + name);
                }

                return group;
            },
            py::arg("name"))
        .def(
            "get_parameter_bool",
            [](const IParametersHandler& impl, const std::string& name) -> bool {
                bool ret;

                if (!impl.getParameter(name, ret))
                {
                    throw py::value_error("Failed to find a parameter that matches the type");
                }

                return ret;
            },
            py::arg("name"))
        .def(
            "get_parameter_int",
            [](const IParametersHandler& impl, const std::string& name) -> int {
                int ret;

                if (!impl.getParameter(name, ret))
                {
                    throw py::value_error("Failed to find a parameter that matches the type");
                }

                return ret;
            },
            py::arg("name"))
        .def(
            "get_parameter_datetime",
            [](const IParametersHandler& impl, const std::string& name) -> std::chrono::nanoseconds {
                std::chrono::nanoseconds ret;

                if (!impl.getParameter(name, ret))
                {
                    throw py::value_error("Failed to find a parameter that matches the type");
                }

                return ret;
            },
            py::arg("name"))
        .def(
            "get_parameter_float",
            [](const IParametersHandler& impl, const std::string& name) -> double {
                double ret;

                if (!impl.getParameter(name, ret))
                {
                    throw py::value_error("Failed to find a parameter that matches the type");
                }

                return ret;
            },
            py::arg("name"))
        .def(
            "get_parameter_string",
            [](const IParametersHandler& impl, const std::string& name) -> std::string {
                std::string ret;

                if (!impl.getParameter(name, ret))
                {
                    throw py::value_error("Failed to find a parameter that matches the type");
                }

                return ret;
            },
            py::arg("name"))
        .def(
            "get_parameter_vector_bool",
            [](const IParametersHandler& impl, const std::string& name) {
                if (std::vector<bool> ret; !impl.getParameter(name, ret))
                {
                    throw py::value_error("Failed to find a parameter that matches the type");
                } else
                {
                    return ret;
                }
            },
            py::arg("name"))
        .def(
            "get_parameter_vector_int",
            [](const IParametersHandler& impl, const std::string& name) {
                if (std::vector<int> ret; !impl.getParameter(name, ret))
                {
                    throw py::value_error("Failed to find a parameter that matches the type");
                } else
                {
                    return ret;
                }
            },
            py::arg("name"))
        .def(
            "get_parameter_vector_float",
            [](const IParametersHandler& impl, const std::string& name) {
                if (std::vector<double> ret; !impl.getParameter(name, ret))
                {
                    throw py::value_error("Failed to find a parameter that matches the type");
                } else
                {
                    return ret;
                }
            },
            py::arg("name"))
        .def(
            "get_parameter_vector_string",
            [](const IParametersHandler& impl, const std::string& name) {
                if (std::vector<std::string> ret; !impl.getParameter(name, ret))
                {
                    throw py::value_error("Failed to find a parameter that matches the type");
                } else
                {
                    return ret;
                }
            },
            py::arg("name"))
        .def(
            "get_parameter_vector_datetime",
            [](const IParametersHandler& impl, const std::string& name) {
                if (std::vector<std::chrono::nanoseconds> ret; !impl.getParameter(name, ret))
                {
                    throw py::value_error("Failed to find a parameter that matches the type");
                } else
                {
                    return ret;
                }
            },
            py::arg("name"))
        .def("clear", &IParametersHandler::clear)
        .def("is_empty", &IParametersHandler::isEmpty)
        .def("__repr__", &IParametersHandler::toString);
}

void CreateStdParameterHandler(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::ParametersHandler;
    py::class_<StdImplementation, //
               std::shared_ptr<StdImplementation>,
               IParametersHandler>(module, "StdParametersHandler")
        .def(py::init());
}

} // namespace ParametersHandler
} // namespace bindings
} // namespace BipedalLocomotion
