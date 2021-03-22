/**
 * @file ParametersHandler.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

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

    py::class_<IParametersHandler, std::shared_ptr<IParametersHandler>>(module,
                                                                        "IParametersHandler");
}

void CreateStdParameterHandler(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::ParametersHandler;

    py::class_<GenericContainer::Vector<const int>>(module, "GenericContainerVectorInt");
    py::class_<GenericContainer::Vector<const double>>(module, "GenericContainerVectorDouble");

    py::class_<StdImplementation, //
               std::shared_ptr<StdImplementation>,
               IParametersHandler>(module, "StdParametersHandler")
        .def(py::init())
        // Note: the order of the following overloads matters!
        .def("set_parameter_bool",
             py::overload_cast<const std::string&, const bool&>(&StdImplementation::setParameter),
             py::arg("name"),
             py::arg("value"))
        .def("set_parameter_int",
             py::overload_cast<const std::string&, const int&>(&StdImplementation::setParameter),
             py::arg("name"),
             py::arg("value"))
        .def("set_parameter_float",
             py::overload_cast<const std::string&, const double&>(&StdImplementation::setParameter),
             py::arg("name"),
             py::arg("value"))
        .def("set_parameter_string",
             py::overload_cast<const std::string&, const std::string&>(
                 &StdImplementation::setParameter),
             py::arg("name"),
             py::arg("value"))
        .def("set_parameter_vector_bool",
             py::overload_cast<const std::string&, const std::vector<bool>&>(
                 &StdImplementation::setParameter),
             py::arg("name"),
             py::arg("value"))
        .def(
            "set_parameter_vector_int",
            [](StdImplementation& impl, const std::string& name, const std::vector<int>& vec) {
                impl.setParameter(name, vec);
            },
            py::arg("name"),
            py::arg("value"))
        .def(
            "set_parameter_vector_float",
            [](StdImplementation& impl, const std::string& name, const std::vector<double>& vec) {
                impl.setParameter(name, vec);
            },
            py::arg("name"),
            py::arg("value"))
        .def(
            "set_parameter_vector_string",
            [](StdImplementation& impl,
               const std::string& name,
               const std::vector<std::string>& vec) { impl.setParameter(name, vec); },
            py::arg("name"),
            py::arg("value"))
        .def("set_group", &StdImplementation::setGroup, py::arg("name"), py::arg("new_group"))
        .def(
            "get_parameter_bool",
            [](const StdImplementation& impl, const std::string& name) -> bool {
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
            [](const StdImplementation& impl, const std::string& name) -> int {
                int ret;

                if (!impl.getParameter(name, ret))
                {
                    throw py::value_error("Failed to find a parameter that matches the type");
                }

                return ret;
            },
            py::arg("name"))
        .def(
            "get_parameter_float",
            [](const StdImplementation& impl, const std::string& name) -> double {
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
            [](const StdImplementation& impl, const std::string& name) -> std::string {
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
            [](const StdImplementation& impl, const std::string& name) {
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
            [](const StdImplementation& impl, const std::string& name) {
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
            [](const StdImplementation& impl, const std::string& name) {
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
            [](const StdImplementation& impl, const std::string& name) {
                if (std::vector<std::string> ret; !impl.getParameter(name, ret))
                {
                    throw py::value_error("Failed to find a parameter that matches the type");
                } else
                {
                    return ret;
                }
            },
            py::arg("name"))
        .def("clear", &StdImplementation::clear)
        .def("is_empty", &StdImplementation::isEmpty)
        .def("__repr__", &StdImplementation::toString);
}

} // namespace ParametersHandler
} // namespace bindings
} // namespace BipedalLocomotion
