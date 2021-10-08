/**
 * @file RosImplementation.tpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_ROS_IMPLEMENTATION_TPP
#define BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_ROS_IMPLEMENTATION_TPP

#include <type_traits>
#include <vector>

#include <BipedalLocomotion/GenericContainer/TemplateHelpers.h>
#include <BipedalLocomotion/ParametersHandler/RosImplementation.h>

namespace BipedalLocomotion
{
namespace ParametersHandler
{

template <typename T>
bool RosImplementation::getParameterPrivate(const std::string& parameterName, T& parameter) const
{
    if constexpr (std::is_scalar<T>::value || is_string<T>::value
                  || std::is_same<T, std::vector<bool>>::value)
    {
        return this->m_handle.getParam(parameterName, parameter);
    } else
    {
        std::vector<typename T::value_type> tempVector;
        bool ok = this->m_handle.getParam(parameterName, tempVector);
        ok = ok && parameter.clone(tempVector);
        return ok;
    }

    return true;
}

template <typename T>
void RosImplementation::setParameterPrivate(const std::string& parameterName, const T& parameter)
{
    if constexpr (std::is_scalar<T>::value || is_string<T>::value
                  || std::is_same<T, std::vector<bool>>::value)
    {
        this->m_handle.setParam(parameterName, parameter);
    } else
    {
        std::vector<typename T::value_type> tempVector;
        tempVector.resize(parameter.size());
        for (int i = 0; i < parameter.size(); i++)
        {
            tempVector[i] = parameter[i];
        }
        this->m_handle.setParam(parameterName, tempVector);
    }
    return;
}

} // namespace ParametersHandler
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_ROS_IMPLEMENTATION_TPP
