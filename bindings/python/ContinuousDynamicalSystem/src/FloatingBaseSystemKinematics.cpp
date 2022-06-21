/**
 * @file FloatingBaseSystemKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>

#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/Integrator.h>

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{

void CreateFloatingBaseSystemKinematics(pybind11::module& module)
{
    using namespace BipedalLocomotion::ContinuousDynamicalSystem;
    namespace py = ::pybind11;

    constexpr auto name = "FloatingBaseSystemKinematics";

    CreateDynamicalSystem<DynamicalSystem<FloatingBaseSystemKinematics>,
                          std::shared_ptr<DynamicalSystem<FloatingBaseSystemKinematics>>> //
        (module, name);

    py::class_<FloatingBaseSystemKinematics,
               DynamicalSystem<FloatingBaseSystemKinematics>,
               std::shared_ptr<FloatingBaseSystemKinematics>>(module, name)
        .def(py::init());

    CreateForwardEulerIntegrator<FloatingBaseSystemKinematics>(module, name);
}
} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion
