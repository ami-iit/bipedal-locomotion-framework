/**
 * @file Spline.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_MATH_SPLINE_H
#define BIPEDAL_LOCOMOTION_BINDINGS_MATH_SPLINE_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Math
{

void CreateSpline(pybind11::module& module);
void CreateZeroOrderSpline(pybind11::module& module);
void CreateLinearSpline(pybind11::module& module);
void CreateCubicSpline(pybind11::module& module);
void CreateQuinticSpline(pybind11::module& module);

} // namespace Math
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_MATH_SPLINE_H
