/**
 * @file YarpImplementation.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <type_traits>

#include <BipedalLocomotionControllers/YarpUtilities/Helper.h>

namespace BipedalLocomotionControllers
{
namespace ParametersHandler
{

template <typename T>
bool YarpImplementation::getParameter(const std::string& parameterName, T& parameter) const
{
    // a scalar element and a strings is retrieved using getElementFromSearchable() function
    if constexpr (std::is_scalar<T>::value || is_string<T>::value)
        return YarpUtilities::getElementFromSearchable(m_container, parameterName, parameter);
    else
        // otherwise it is considered as a vector
        return YarpUtilities::getVectorFromSearchable(m_container, parameterName, parameter);
}

template <typename T>
void YarpImplementation::setParameter(const std::string& parameterName, const T& parameter)
{
    // a scalar element and a strings is retrieved using getElementFromSearchable() function
    if constexpr (std::is_scalar<T>::value || is_string<T>::value)
    {
        m_container.put(parameterName, parameter);
    } else
    {
        yarp::os::Value yarpValue;
        auto list = yarpValue.asList();

        for (const auto& v : parameter)
            list->add(yarp::os::Value(v));

        m_container.put(parameterName, yarpValue);
    }
}

} // namespace ParametersHandler
} // namespace BipedalLocomotionControllers
