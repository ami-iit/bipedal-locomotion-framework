/**
 * @file FloatingBaseSystemAccelerationKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemAccelerationKinematics.h>

#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/FloatingBaseSystemAccelerationKinematics.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/Integrator.h>

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{

void CreateFloatingBaseSystemAccelerationKinematics(pybind11::module& module)
{
    using namespace BipedalLocomotion::ContinuousDynamicalSystem;
    namespace py = ::pybind11;

    constexpr auto name = "FloatingBaseSystemAccelerationKinematics";

    CreateDynamicalSystem<DynamicalSystem<FloatingBaseSystemAccelerationKinematics>,
                          std::shared_ptr<DynamicalSystem<FloatingBaseSystemAccelerationKinematics>>> //
        (module, name);

    py::class_<FloatingBaseSystemAccelerationKinematics,
               DynamicalSystem<FloatingBaseSystemAccelerationKinematics>,
               std::shared_ptr<FloatingBaseSystemAccelerationKinematics>>(module, name)
        .def(py::init());

    CreateForwardEulerIntegrator<FloatingBaseSystemAccelerationKinematics>(module, name);
}
} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion
