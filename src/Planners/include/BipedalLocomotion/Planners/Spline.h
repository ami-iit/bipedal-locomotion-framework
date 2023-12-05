/**
 * @file Spline.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_SPLINE_H
#define BIPEDAL_LOCOMOTION_PLANNERS_SPLINE_H

#include <BipedalLocomotion/Math/Spline.h>

namespace BipedalLocomotion
{
namespace Planners
{

/**
 * SplineState implements the state of the spline.
 * @warning SplineState is deprecated please use BipedalLocomotion::Math::TrajectoryPoint
 */
using SplineState [[deprecated("BipedalLocomotion::Planners::SplineState is deprecated please "
                               "use BipedalLocomotion::Math::TrajectoryPoint")]]
= Math::TrajectoryPoint<Eigen::VectorXd>;

/**
 * Spline implements a basic spline.
 * @warning Spline is deprecated please use BipedalLocomotion::Math::Spline
 */
using Spline [[deprecated("BipedalLocomotion::Planners::Spline is deprecated please "
                          "use BipedalLocomotion::Math::Spline")]]
= Math::Spline<Eigen::VectorXd>;

} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_SPLINE_H
