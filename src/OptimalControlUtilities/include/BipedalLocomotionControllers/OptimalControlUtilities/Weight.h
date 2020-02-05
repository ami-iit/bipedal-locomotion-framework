/**
 * @file Weight.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_WEIGHT_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_WEIGHT_H

#include <iDynTree/Core/VectorDynSize.h>

namespace BipedalLocomotionControllers
{
namespace OptimalControlUtilities
{
template <typename T> class Weight
{
    T m_rawWeight; /**< raw weight */
    double m_offset{0}; /**< Offset of the weight i.e. Weight = scaling * raw_weight + offset */
    double m_scaling{1}; /**< Scaling factor of the weight i.e. Weight = scaling * raw_weight +
                          offset*/

public:
    Weight(const T& rawWeight, const double& weightOffset, const double& weightScaling) noexcept;

    Weight(const T& rawWeight) noexcept;

    T getWeight() const noexcept;

    void setScaling(const double& scaling) noexcept;

    void setOffset(const double& offset) noexcept;

    void setRawWeight(const T& rawWeight) noexcept;

    constexpr static Weight<T> Zero(size_t size = 1) noexcept;
};
} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers

#include "Weight.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_WEIGHT_H
