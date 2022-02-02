/**
 * @file IWeightProvider.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_IWEIGHT_PROVIDER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_IWEIGHT_PROVIDER_H

#include <Eigen/Dense>

namespace BipedalLocomotion
{
namespace System
{

/**
 * IWeightProvider describes the interface for a provider that produces the weight for a
 * BipedalLocomotion::System::ILinearTaskSolver
 */
struct IWeightProvider
{

    /**
     * Get the weight associated to the provider
     * @return A vector representing the diagonal matrix of the weight
     */
    virtual Eigen::Ref<const Eigen::VectorXd> getWeight() const = 0;

    /**
     * Destructor.
     */
    virtual ~IWeightProvider() = default;
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_IWEIGHT_PROVIDER_H
