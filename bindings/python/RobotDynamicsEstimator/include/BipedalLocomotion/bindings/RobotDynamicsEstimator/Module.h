/**
 * @file Module.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_ROBOT_DYNAMICS_ESTIMATOR_MODULE_H
#define BIPEDAL_LOCOMOTION_BINDINGS_ROBOT_DYNAMICS_ESTIMATOR_MODULE_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotDynamicsEstimator
{

void CreateModule(pybind11::module& module);

} // namespace RobotDynamicsEstimator
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_ROBOT_DYNAMICS_ESTIMATOR_MODULE_H
