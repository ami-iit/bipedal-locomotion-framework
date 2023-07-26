/**
 * @file CentroidalDynamics.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/CentroidalDynamics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>

#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/CentroidalDynamics.h>
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

void CreateCentroidalDynamics(pybind11::module& module)
{
    using namespace BipedalLocomotion::ContinuousDynamicalSystem;
    namespace py = ::pybind11;

    constexpr auto name = "CentroidalDynamics";

    CreateDynamicalSystem<DynamicalSystem<CentroidalDynamics>,
                          std::shared_ptr<DynamicalSystem<CentroidalDynamics>>> //
        (module, name);

    py::class_<CentroidalDynamics,
               DynamicalSystem<CentroidalDynamics>,
               std::shared_ptr<CentroidalDynamics>>(module, name)
        .def(py::init());

    CreateForwardEulerIntegrator<CentroidalDynamics>(module, name);
    CreateRK4Integrator<CentroidalDynamics>(module, name);
}
} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion
