/**
 * @file CentroidalDynamics.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_CONTINUOUS_DYNAMICAL_SYSTEM_CENTROIDAL_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_BINDINGS_CONTINUOUS_DYNAMICAL_SYSTEM_CENTROIDAL_DYNAMICS_H

#include <memory>

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{

void CreateCentroidalDynamics(pybind11::module& module);

} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_CONTINUOUS_DYNAMICAL_SYSTEM_CENTROIDAL_DYNAMICS_H
