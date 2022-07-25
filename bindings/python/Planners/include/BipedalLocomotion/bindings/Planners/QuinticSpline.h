/**
 * @file QuinticSpline.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_PLANNERS_QUINTIC_SPLINE_H
#define BIPEDAL_LOCOMOTION_BINDINGS_PLANNERS_QUINTIC_SPLINE_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Planners
{

void CreateQuinticSpline(pybind11::module& module);

} // namespace Planners
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_PLANNERS_QUINTIC_SPLINE_H
