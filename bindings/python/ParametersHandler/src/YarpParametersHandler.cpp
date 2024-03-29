/**
 * @file YarpParametersHandler.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/bindings/ParametersHandler/YarpParametersHandler.h>

#include <yarp/os/ResourceFinder.h>

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
        .def("set_from_filename",
             [](YarpImplementation& impl, const std::string& filename) -> bool {
                 const auto filePath
                     = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(
                         filename);
                 return impl.setFromFile(filePath);
             })
        .def("set_from_file_path", &YarpImplementation::setFromFile, py::arg("file_path"));
}

} // namespace ParametersHandler
} // namespace bindings
} // namespace BipedalLocomotion
