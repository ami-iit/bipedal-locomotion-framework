/**
 * @file StdImplementation.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_STD_IMPLEMENTATION_TPP
#define BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_STD_IMPLEMENTATION_TPP

#include <type_traits>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

namespace BipedalLocomotion
{
namespace ParametersHandler
{

template <typename T>
bool StdImplementation::getParameterPrivate(const std::string& parameterName, T& parameter) const
{
    auto parameterAny = m_map.find(parameterName);
    if (parameterAny == m_map.end())
    {
        std::cerr << "[StdImplementation::getParameterPrivate] Parameter named " << parameterName
                  << " not found." << std::endl;
        return false;
    }

    if constexpr (std::is_scalar<T>::value || is_string<T>::value)
    {
        try
        {
            parameter = std::any_cast<T>(parameterAny->second);
        } catch (const std::bad_any_cast& exception)
        {
            std::cerr << "[StdImplementation::getParameterPrivate] The type of the parameter "
                         "named "
                      << parameterName << " is different from the one expected" << std::endl;
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
            std::cerr << "[StdImplementation::getParameterPrivate] The type of the parameter "
                         "named "
                      << parameterName << " is different from the one expected" << std::endl;
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
                    std::cerr << "[StdImplementation::getParameterPrivate] Unable to resize "
                              << type_name<T>() << "List size: " << castedParameter.size()
                              << ". Vector size: " << parameter.size() << std::endl;
                    return false;
                }
            } else if constexpr (is_resizable<T>::value)
                parameter.resize(castedParameter.size());
            else
            {
                std::cerr << "[StdImplementation::getParameterPrivate] The size of the "
                             "vector does not match with the size of the list. List size: "
                          << castedParameter.size() << ". Vector size: " << parameter.size()
                          << std::endl;
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
