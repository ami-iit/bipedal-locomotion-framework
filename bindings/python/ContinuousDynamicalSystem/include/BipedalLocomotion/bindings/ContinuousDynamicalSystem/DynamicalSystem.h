/**
 * @file DynamicalSystem.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_CONTINUOUS_DYNAMICAL_SYSTEM_DYNAMICAL_SYSTEM_H
#define BIPEDAL_LOCOMOTION_BINDINGS_CONTINUOUS_DYNAMICAL_SYSTEM_DYNAMICAL_SYSTEM_H

#include <memory>

#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{

template <typename _DynamicalSystem, typename... _Args>
void CreateDynamicalSystem(pybind11::class_<_DynamicalSystem, _Args...>& pyClass)
{
    namespace py = ::pybind11;

    pyClass.def(pybind11::init<>())
        .def(
            "initialize",
            [](_DynamicalSystem& impl,
               std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler) {
                return impl.initialize(handler);
            },
            py::arg("param_handler"))
        .def("set_state", &_DynamicalSystem::setState, py::arg("state"))
        .def("get_state", &_DynamicalSystem::getState)
        .def("set_control_input", &_DynamicalSystem::setControlInput, py::arg("control_input"));
}

} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_CONTINUOUS_DYNAMICAL_SYSTEM_DYNAMICAL_SYSTEM_H
