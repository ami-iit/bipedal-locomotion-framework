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
    if constexpr (std::is_scalar<T>::value || std::is_same<T, std::string>::value)
        return YarpUtilities::getElementFromSearchable(m_searchable, parameterName, parameter);
    else
        // otherwise it is considered as a vector
        return YarpUtilities::getVectorFromSearchable(m_searchable, parameterName, parameter);
}
} // namespace ParametersHandler
} // namespace BipedalLocomotionControllers
