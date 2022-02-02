/**
 * @file WeightProvider.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <memory>
#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/ConstantWeightProvider.h>
#include <BipedalLocomotion/System/IWeightProvider.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{

void CreateIWeightProvider(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;

    py::class_<IWeightProvider, std::shared_ptr<IWeightProvider>>(module, "IWeightProvider")
        .def("get_weight", &IWeightProvider::getWeight);
}

void CreateConstantWeightProvider(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;

    py::class_<ConstantWeightProvider,
               IWeightProvider,
               std::shared_ptr<ConstantWeightProvider>>(module, "ConstantWeightProvider")
        .def(py::init())
        .def(py::init<Eigen::Ref<const Eigen::VectorXd>>(), py::arg("weight"))
        .def("initialize",
             [](ConstantWeightProvider& impl,
                std::shared_ptr<ParametersHandler::IParametersHandler> handler) -> bool {
                 return impl.initialize(handler);
             })
        .def_readwrite("weight", &ConstantWeightProvider::weight);
}

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
