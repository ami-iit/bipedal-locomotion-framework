/**
 * @file WeightProvider.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_WEIGHT_PROVIDER_H
#define BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_WEIGHT_PROVIDER_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{

namespace System
{

void CreateIWeightProvider(pybind11::module& module);
void CreateConstantWeightProvider(pybind11::module& module);

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_WEIGHT_PROVIDER_H
