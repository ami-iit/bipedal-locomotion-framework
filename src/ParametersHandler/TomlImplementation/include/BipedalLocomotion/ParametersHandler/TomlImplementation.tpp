/**
 * @file TomlImplementation.tpp
 * @authors Giulio Romualdi
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_TOML_IMPLEMENTATION_TPP
#define BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_TOML_IMPLEMENTATION_TPP

#include <type_traits>

#include <BipedalLocomotion/ParametersHandler/TomlImplementation.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/GenericContainer/TemplateHelpers.h>

namespace BipedalLocomotion
{
namespace ParametersHandler
{

template <typename T>
bool TomlImplementation::getParameterPrivate(const std::string& parameterName, T& parameter) const
{
    constexpr auto logPrefix = "[TomlImplementation::getParameterPrivate]";
    auto paramToml = m_container[parameterName];

    if constexpr (std::is_scalar<T>::value || is_string<T>::value)
    {
        if constexpr (std::is_integral<T>() && !std::is_same_v<T, bool>)
        {
            if (!paramToml.is_integer())
            {
                BipedalLocomotion::log()->debug("{} The type of the parameter named {} is "
                                                "different from expected one.",
                                                logPrefix,
                                                parameterName);
                return false;
            }
        } else
        {
            if (!paramToml.is<T>())
            {
                BipedalLocomotion::log()->debug("{} The type of the parameter named {} is "
                                                "different from expected one.",
                                                logPrefix,
                                                parameterName);

                return false;
            }
        }
        parameter = paramToml.value<T>().value();
        return true;
    } else // otherwise it is considered as a vector
    {
        using elementType = typename T::value_type;

        if (!paramToml.is_array())
        {
            BipedalLocomotion::log()->debug("{} The parameter named {} is not an array.",
                                            logPrefix,
                                            parameterName);
            return false;
        }

        const toml::array& array = *paramToml.as_array();

        // check the size
        if (array.size() != parameter.size())
        {
            // If the vector can be resize, let resize it. Otherwise it is a fix-size vector and
            // the dimensions has to be the same of list
            if constexpr (GenericContainer::is_vector<T>::value)
            {
                if (!parameter.resizeVector(array.size()))
                {
                    BipedalLocomotion::log()->debug("{} Unable to resize {} List size: {}. Vector "
                                                    "size: {}.",
                                                    logPrefix,
                                                    type_name<T>(),
                                                    array.size(),
                                                    parameter.size());

                    return false;
                }
            } else if constexpr (is_resizable<T>::value)
                parameter.resize(array.size());
            else
            {
                BipedalLocomotion::log()->debug("{} The size of the vector does not match with the "
                                                "size of the list. "
                                                "List size: {}. Vector size: {}.",
                                                logPrefix,
                                                array.size(),
                                                parameter.size());
                return false;
            }
        }

        // check if the array is homogeneous
        if constexpr (std::is_same_v<elementType, int>)
        {
            if (!array.is_homogeneous(toml::node_type::integer))
            {
                BipedalLocomotion::log()->debug("{} The type of the parameter named {} is "
                                                "different from expected one.",
                                                logPrefix,
                                                parameterName);

                return false;
            }
        } else
        {
            if (!array.is_homogeneous<elementType>())
            {
                BipedalLocomotion::log()->debug("{} The type of the parameter named {} is "
                                                "different from expected one.",
                                                logPrefix,
                                                parameterName);

                return false;
            }
        }

        for (int i = 0; i < array.size(); i++)
        {
            parameter[i] = array[i].value<elementType>().value();
        }

        return true;
    }
}

template <typename T>
void TomlImplementation::setParameterPrivate(const std::string& parameterName, const T& parameter)
{
    BipedalLocomotion::log()->error("[TomlImplementation::setParameterPrivate] This function is "
                                    "not implemented");
}

} // namespace ParametersHandler
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_TOML_IMPLEMENTATION_TPP
