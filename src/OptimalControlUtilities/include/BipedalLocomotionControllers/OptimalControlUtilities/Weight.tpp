/**
 * @file Weight.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

namespace BipedalLocomotionControllers
{
namespace OptimalControlUtilities
{
template <typename T>
Weight<T>::Weight(const T& rawWeight,
                  const double& offset,
                  const double& scaling) noexcept
    : m_rawWeight(rawWeight)
    , m_offset(offset)
    , m_scaling(scaling)
{
}

template <typename T>
Weight<T>::Weight(const T& rawWeight) noexcept
    : m_rawWeight(rawWeight)
{
}

template <typename T> void Weight<T>::setScaling(const double& scaling) noexcept
{
    m_scaling = scaling;
}

template <typename T> void Weight<T>::setOffset(const double& offset) noexcept
{
    m_offset = offset;
}

template <typename T> void Weight<T>::setRawWeight(const T& rawWeight) noexcept
{
    m_rawWeight = rawWeight;
}

template <typename T> T Weight<T>::getWeight() const noexcept
{
    T weight(m_rawWeight);

    // if it is a number we cannot iterate
    if constexpr (std::is_arithmetic_v<T>)
        weight = weight * m_scaling + m_offset;
    else
    {
        // evaluate the weight
        for (auto& element : weight)
            element = element * m_scaling + m_offset;
    }

    return weight;
}

template <typename T>
constexpr Weight<T> Weight<T>::Zero(size_t size) noexcept
{
    T rawWeight;
    if constexpr (std::is_arithmetic_v<T>)
        rawWeight = 0;
    else
    {
        rawWeight.resize(size);
        // evaluate the weight
        for (auto& element : rawWeight)
            element = 0;
    }

    Weight<T> weight(rawWeight);
    return weight;
}

} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers
