/**
 * @file Constants.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_MATH_CONSTANTS_H
#define BIPEDAL_LOCOMOTION_MATH_CONSTANTS_H

namespace BipedalLocomotion
{
namespace Math
{
/**
 * The standard acceleration due to gravity (or standard acceleration of free fall), sometimes
 * abbreviated as standard gravity, is the nominal gravitational
 * acceleration of an object in a vacuum near the surface of the Earth.
 */
constexpr double StandardAccelerationOfGravitation = 9.80665;

/**
 * The Absolute tolerance used to consider two double equal.
 */
constexpr double AbsoluteEqualityDoubleTolerance = 1e-8;

} // namespace Math
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATH_CONSTANTS_H
