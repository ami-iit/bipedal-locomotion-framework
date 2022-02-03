
/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/Module.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/MultiStateWeightProvider.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "ContinousDynamicalSystem module contains the bindings for "
                   "BipedalLocomotion::ContinousDynamicalSystem";

    CreateMultiStateWeightProvider(module);
}
} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion
