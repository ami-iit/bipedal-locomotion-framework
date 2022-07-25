/**
 * @file StdImplementation.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_STD_IMPLEMENTATION_TPP
#define BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_STD_IMPLEMENTATION_TPP

#include <type_traits>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
namespace ParametersHandler
{

template <typename T>
bool StdImplementation::getParameterPrivate(const std::string& parameterName, T& parameter) const
{
    constexpr auto logPrefix = "[StdImplementation::getParameterPrivate]";

    auto parameterAny = m_map.find(parameterName);
    if (parameterAny == m_map.end())
    {
        log()->debug("{} Parameter named '{}' not found.", logPrefix, parameterName);
        return false;
    }

    if constexpr (std::is_scalar<T>::value || is_string<T>::value)
    {
        try
        {
            parameter = std::any_cast<T>(parameterAny->second);
        } catch (const std::bad_any_cast& exception)
        {
            log()->debug("{} The type of the parameter named '{}' is different from the one "
                         "expected.",
                         logPrefix,
                         parameterName);
            return false;
        }
    } else
    {
        using elementType = typename T::value_type;
        std::vector<elementType> castedParameter;
        try
        {
            castedParameter = std::any_cast<std::vector<elementType>>(parameterAny->second);
        } catch (const std::bad_any_cast& exception)
        {
            log()->debug("{} The type of the parameter named {} is different from the one "
                         "expected.",
                         logPrefix,
                         parameterName);
            return false;
        }

        if (castedParameter.size() != parameter.size())
        {
            // If the vector can be resize, let resize it. Otherwise it is a fix-size vector and
            // the dimensions has to be the same of list
            if constexpr (GenericContainer::is_vector<T>::value)
            {
                if (!parameter.resizeVector(castedParameter.size()))
                {
                    log()->debug("{} Unable to resize {} List size: {}. Vector size: {}.",
                                 logPrefix,
                                 type_name<T>(),
                                 castedParameter.size(),
                                 parameter.size());

                    return false;
                }
            } else if constexpr (is_resizable<T>::value)
                parameter.resize(castedParameter.size());
            else
            {
                log()->debug("{} The size of the vector does not match with the size of the list. "
                             "List size: {}. Vector size: {}.",
                             logPrefix,
                             castedParameter.size(),
                             parameter.size());
                return false;
            }
        }

        for (std::size_t index = 0; index < parameter.size(); index++)
            parameter[index] = castedParameter[index];
    }
    return true;
}

template <typename T>
void StdImplementation::setParameterPrivate(const std::string& parameterName, const T& parameter)
{
    // a scalar element and a strings is retrieved using getElementFromSearchable() function
    if constexpr (std::is_scalar<T>::value || is_string<T>::value)
        m_map[parameterName] = parameter;
    else
    {
        using elementType = typename T::value_type;
        std::vector<elementType> tempParameter(parameter.size());

        for (std::size_t index = 0; index < parameter.size(); index++)
            tempParameter[index] = parameter[index];

        m_map[parameterName] = std::make_any<std::vector<elementType>>(tempParameter);
    }
}

} // namespace ParametersHandler
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_STD_IMPLEMENTATION_TPP
