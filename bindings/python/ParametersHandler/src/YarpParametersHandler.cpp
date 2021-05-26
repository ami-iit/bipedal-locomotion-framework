/**
 * @file YarpParametersHandler.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/bindings/ParametersHandler/YarpParametersHandler.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ParametersHandler
{

void CreateYarpParameterHandler(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::ParametersHandler;
    py::class_<YarpImplementation, //
               std::shared_ptr<YarpImplementation>,
               IParametersHandler>(module, "YarpParametersHandler")
        .def(py::init())
        .def("set_from_file", &YarpImplementation::setFromFile, py::arg("Filename"));
}

} // namespace ParametersHandler
} // namespace bindings
} // namespace BipedalLocomotion
