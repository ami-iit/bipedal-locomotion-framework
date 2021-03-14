/**
 * @file LeggedOdometry.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_FLOATING_BASE_ESTIMATORS_LEGGED_ODOMETRY_H
#define BIPEDAL_LOCOMOTION_BINDINGS_FLOATING_BASE_ESTIMATORS_LEGGED_ODOMETRY_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace FloatingBaseEstimators
{

void CreateLeggedOdometry(pybind11::module& module);

} // namespace FloatingBaseEstimators
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_FLOATING_BASE_ESTIMATORS_LEGGED_ODOMETRY_H
