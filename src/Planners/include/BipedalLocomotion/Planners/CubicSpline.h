/**
 * @file Spline.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_CUBIC_SPLINE_H
#define BIPEDAL_LOCOMOTION_PLANNERS_CUBIC_SPLINE_H

#include <BipedalLocomotion/Math/CubicSpline.h>

namespace BipedalLocomotion
{
namespace Planners
{

/**
 * CubicSpline implements a cubic spline.
 * @warning CubicSpline is deprecated please use BipedalLocomotion::Math::CubicSpline
 */
using CubicSpline [[deprecated("BipedalLocomotion::Planners::CubicSpline is deprecated please use "
                               "BipedalLocomotion::Math::CubicSpline")]]
= Math::CubicSpline<Eigen::VectorXd>;

} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_CUBIC_SPLINE_H
