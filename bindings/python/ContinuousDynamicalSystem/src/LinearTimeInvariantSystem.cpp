/**
 * @file LinearTimeInvariantSystem.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>

#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/Integrator.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{

void CreateLinearTimeInvariantSystem(pybind11::module& module)
{
    using namespace BipedalLocomotion::ContinuousDynamicalSystem;
    namespace py = ::pybind11;

    constexpr auto name = "LinearTimeInvariantSystem";

    CreateDynamicalSystem<DynamicalSystem<LinearTimeInvariantSystem>,
                          std::shared_ptr<DynamicalSystem<LinearTimeInvariantSystem>>> //
        (module, name);

    py::class_<LinearTimeInvariantSystem,
               DynamicalSystem<LinearTimeInvariantSystem>,
               std::shared_ptr<LinearTimeInvariantSystem>>(module, name)
        .def(py::init())
        .def("set_system_matrices",
             &LinearTimeInvariantSystem::setSystemMatrices,
             py::arg("A"),
             py::arg("B"));

    CreateForwardEulerIntegrator<LinearTimeInvariantSystem>(module, name);
    CreateRK4Integrator<LinearTimeInvariantSystem>(module, name);
}
} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion
