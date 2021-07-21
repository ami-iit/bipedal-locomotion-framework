/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/Module.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "continuous_dynamical_system module contains the bindings for "
                   "BipedalLocomotion::ContinuousDynamicalSystem";

    CreateLinearTimeInvariantSystem(module);
}
} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion
