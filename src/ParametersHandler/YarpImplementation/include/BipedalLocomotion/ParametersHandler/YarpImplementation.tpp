/**
 * @file YarpImplementation.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_YARP_IMPLEMENTATION_TPP
#define BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_YARP_IMPLEMENTATION_TPP

#include <type_traits>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/YarpUtilities/Helper.h>

namespace BipedalLocomotion
{
namespace ParametersHandler
{

template <typename T>
bool YarpImplementation::getParameterPrivate(const std::string& parameterName, T& parameter) const
{
    if (m_lists.find(parameterName) != m_lists.end()) // A list is called with the same name of the
                                                      // parameter we are searching
    {
        return m_lists.at(parameterName)->getParameter(parameterName, parameter);
    } else
    {
        // a scalar element and a strings is retrieved using getElementFromSearchable() function
        if constexpr (std::is_scalar<T>::value || is_string<T>::value)
            return YarpUtilities::getElementFromSearchable(m_container, parameterName, parameter);
        else
            // otherwise it is considered as a vector
            return YarpUtilities::getVectorFromSearchable(m_container, parameterName, parameter);
    }
}

template <typename T>
void YarpImplementation::setParameterPrivate(const std::string& parameterName, const T& parameter)
{
    // a scalar element and a strings is retrieved using getElementFromSearchable() function
    if constexpr (std::is_scalar<T>::value || is_string<T>::value)
    {
        yarp::os::Value& check = m_container.find(parameterName);
        if (check.isNull())
        {
            yarp::os::Value newVal;
            yarp::os::Bottle* list = newVal.asList();
            list->add(yarp::os::Value(parameterName));

            // if the parameter is a boolean we cannot use the usual way to add the new parameter
            // Please check https://github.com/robotology/yarp/issues/2584#issuecomment-847778679
            if constexpr (std::is_same<T, bool>::value)
            {
                if (parameter)
                {
                    list->add(yarp::os::Value::makeValue("true"));
                } else
                {
                    list->add(yarp::os::Value::makeValue("false"));
                }
            } else
            {
                list->add(yarp::os::Value(parameter));
            }

            m_container.add(newVal);
        } else
        {
            check = yarp::os::Value(parameter);
        }
    } else
    {
        yarp::os::Value yarpValue;
        auto property = yarpValue.asList();
        property->add(yarp::os::Value(parameterName));

        yarp::os::Value yarpNewList;
        auto newList = yarpNewList.asList();

        for (const auto& element : parameter)
        {
            // if the parameter is a boolean we cannot use the usual way to add the new parameter
            // Please check https://github.com/robotology/yarp/issues/2584#issuecomment-847778679
            if constexpr (std::is_same<typename std::remove_cv_t<
                                           typename std::remove_reference_t<decltype(element)>>,
                                       bool>::value)
            {
                if (element)
                {
                    newList->add(yarp::os::Value::makeValue("true"));
                } else
                {
                    newList->add(yarp::os::Value::makeValue("false"));
                }
            } else
            {
                newList->add(yarp::os::Value(element));
            }
        }

        property->add(yarpNewList);

        m_lists[parameterName] = std::make_shared<YarpImplementation>(yarpValue);
    }
}

} // namespace ParametersHandler
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_YARP_IMPLEMENTATION_TPP
