/**
 * @file EMAWithLimits.h
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_JOINT_LEVEL_CONTROLLERS_EMA_WITH_LIMITS_H
#define BIPEDAL_LOCOMOTION_BINDINGS_JOINT_LEVEL_CONTROLLERS_EMA_WITH_LIMITS_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace JointLevelControllers
{

void CreateEMAWithLimits(pybind11::module& module);

} // namespace JointLevelControllers
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_JOINT_LEVEL_CONTROLLERS_EMA_WITH_LIMITS_H
