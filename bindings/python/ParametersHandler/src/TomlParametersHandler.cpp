/**
 * @file TomlParametersHandler.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/TomlImplementation.h>
#include <BipedalLocomotion/bindings/ParametersHandler/TomlParametersHandler.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ParametersHandler
{

void CreateTomlParameterHandler(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::ParametersHandler;
    py::class_<TomlImplementation, //
               std::shared_ptr<TomlImplementation>,
               IParametersHandler>(module, "TomlParametersHandler")
        .def(py::init())
        .def("set_from_file", &TomlImplementation::setFromFile, py::arg("file_name"));
}

} // namespace ParametersHandler
} // namespace bindings
} // namespace BipedalLocomotion
