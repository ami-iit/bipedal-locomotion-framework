/**
 * @file WeightProvider.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_WEIGHT_PROVIDER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_WEIGHT_PROVIDER_H

#include <BipedalLocomotion/System/Source.h>

namespace BipedalLocomotion
{
namespace System
{

/**
 * WeightProvider describes the provider for a weight.
 */
struct WeightProvider : public Source<Eigen::VectorXd>
{
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_CONSTANT_WEIGHT_PROVIDER_H
