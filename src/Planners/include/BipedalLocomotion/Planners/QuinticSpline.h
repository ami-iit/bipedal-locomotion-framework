/**
 * @file QuinticSpline.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_QUINTIC_SPLINE_H
#define BIPEDAL_LOCOMOTION_PLANNERS_QUINTIC_SPLINE_H

#include <BipedalLocomotion/Math/QuinticSpline.h>

namespace BipedalLocomotion
{
namespace Planners
{

/**
 * QuinticSpline implements a quintic spline.
 * @warning QuinticSpline is deprecated please use BipedalLocomotion::Math::QuinticSpline
 */
using QuinticSpline [[deprecated("BipedalLocomotion::Planners::QuinticSpline is deprecated please "
                                 "use BipedalLocomotion::Math::QuinticSpline")]]
= Math::QuinticSpline<Eigen::VectorXd>;

} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_QUINTIC_SPLINE_H
