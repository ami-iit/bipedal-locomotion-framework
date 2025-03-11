/**
 * @file FloatingBaseSystemVelocityKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemVelocityKinematics.h>

#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/FloatingBaseSystemVelocityKinematics.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/Integrator.h>

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{

void CreateFloatingBaseSystemVelocityKinematics(pybind11::module& module)
{
    using namespace BipedalLocomotion::ContinuousDynamicalSystem;
    namespace py = ::pybind11;

    constexpr auto name = "FloatingBaseSystemVelocityKinematics";

    CreateDynamicalSystem<DynamicalSystem<FloatingBaseSystemVelocityKinematics>,
                          std::shared_ptr<DynamicalSystem<FloatingBaseSystemVelocityKinematics>>> //
        (module, name);

    py::class_<FloatingBaseSystemVelocityKinematics,
               DynamicalSystem<FloatingBaseSystemVelocityKinematics>,
               std::shared_ptr<FloatingBaseSystemVelocityKinematics>>(module, name)
        .def(py::init());

    CreateForwardEulerIntegrator<FloatingBaseSystemVelocityKinematics>(module, name);
}
} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion
