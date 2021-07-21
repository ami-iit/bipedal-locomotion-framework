/**
 * @file LinearTimeInvariantSystem.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_CONTINUOUS_DYNAMICAL_SYSTEM_LINEAR_TIME_INVARIANT_SYSTEM_H
#define BIPEDAL_LOCOMOTION_BINDINGS_CONTINUOUS_DYNAMICAL_SYSTEM_LINEAR_TIME_INVARIANT_SYSTEM_H

#include <memory>

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{

void CreateLinearTimeInvariantSystem(pybind11::module& module);

} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_CONTINUOUS_DYNAMICAL_SYSTEM_LINEAR_TIME_INVARIANT_SYSTEM_H
