/**
 * @file WeightProvider.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <memory>
#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/ConstantWeightProvider.h>
#include <BipedalLocomotion/System/WeightProvider.h>

#include <BipedalLocomotion/bindings/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{

void CreateWeightProvider(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;

    py::class_<WeightProvider, std::shared_ptr<WeightProvider>>(module, "WeightProvider")
        .def(
            "initialize",
            [](WeightProvider& impl,
               std::shared_ptr<const ::BipedalLocomotion::ParametersHandler::IParametersHandler>
                   handler) -> bool { return impl.initialize(handler); },
            py::arg("handler"))
        .def("advance", &WeightProvider::advance)
        .def("close", &WeightProvider::close)
        .def("get_output", &WeightProvider::getOutput)
        .def("is_output_valid", &WeightProvider::isOutputValid);
}

void CreateConstantWeightProvider(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;

    // ::BipedalLocomotion::System::Source<Eigen::VectorXd> has been registered in
    // BipedalLocomotion::bindigs::System::CreateSharedSource()
    py::class_<ConstantWeightProvider, //
               WeightProvider,
               std::shared_ptr<ConstantWeightProvider>>(module, "ConstantWeightProvider")
        .def(py::init())
        .def(py::init<const Eigen::VectorXd&>());
}

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
