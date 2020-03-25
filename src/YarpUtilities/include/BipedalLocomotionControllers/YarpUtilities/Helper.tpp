/**
 * @file Helper.tpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// std
#include <iostream>
#include <type_traits>
//GenericContainer
#include <BipedalLocomotionControllers/GenericContainer/TemplateHelpers.h>
#include <BipedalLocomotionControllers/GenericContainer/Vector.h>


// clang-format off

#define YARP_UTILITES_GET_ELEMENT_TYPE(type)                            \
    ((std::is_same<type, double>::value) ? "double" :                   \
    ((std::is_same<type, int>::value) ? "int" :                         \
    ((std::is_same<type, std::string>::value) ? "string" :              \
    ((std::is_same<type, bool>::value) ? "bool" :                       \
     "undefined" ))))

#define YARP_UTILITES_CHECK_ELEMENT_SUPPORT(type)                       \
    ((std::is_same<type, double>::value) ? true :                       \
    ((std::is_same<type, int>::value) ? true :                          \
    ((std::is_same<type, std::string>::value) ? true :                  \
    ((std::is_same<type, bool>::value) ? true :                         \
     false ))))

#define YARP_UTILITES_GET_CHECKER_NAME(type)                                                  \
    ((std::is_same<type, int>::value) ? &yarp::os::Value::isInt :                             \
    ((std::is_same<type, double>::value) ? &yarp::os::Value::isDouble :                       \
    ((std::is_same<type, std::string>::value) ? &yarp::os::Value::isString :                  \
    ((std::is_same<type, bool>::value) ? &yarp::os::Value::isBool :                           \
     &yarp::os::Value::isDouble ))))

// clang-format on

namespace BipedalLocomotionControllers
{

/**
 * Helper for YARP library.
 */
namespace YarpUtilities
{

template <typename T> T convertValue(const yarp::os::Value& value)
{
    static_assert(dependent_false<T>::value,
                  "[BipedalLocomotionControllers::YarpUtilities::convertValue] The non specialized "
                  "version has not been implemented");

    return T();
}

template <typename T>
bool getElementFromSearchable(const yarp::os::Searchable& config,
                              const std::string& key,
                              T& element)
{

    static_assert(YARP_UTILITES_CHECK_ELEMENT_SUPPORT(T),
                  "[BipedalLocomotionControllers::YarpUtilities::getElementFromSearchable] The "
                  "function getElementFromSearchable() cannot be called with the desired "
                  "element type");

    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getElementFromSearchable] "
                     "Missing field named "
                  << key << std::endl;
        return false;
    }

    if (!(value->*YARP_UTILITES_GET_CHECKER_NAME(T))())
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getElementFromSearchable] The "
                     "value named "
                  << key << " is not a " << YARP_UTILITES_GET_ELEMENT_TYPE(T) << "." << std::endl;
        return false;
    }

    element = convertValue<T>(*value);
    return true;
}

template <typename T>
bool getVectorFromSearchable(const yarp::os::Searchable& config, const std::string& key, T& vector)
{

    using elementType = typename std::pointer_traits<decltype(vector.data())>::element_type;

    static_assert(YARP_UTILITES_CHECK_ELEMENT_SUPPORT(elementType),
                  "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] The "
                  "function getElementFromSearchable() cannot be called with the desired "
                  "element type");

    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] "
                     "Missing field "
                  << key << std::endl;
        return false;
    }

    if (value->isNull())
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] Empty "
                     "input value named "
                  << key << std::endl;
        return false;
    }

    if (!value->isList())
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] The "
                     "value named "
                  << key << "is not associated to a list." << std::endl;
        return false;
    }

    yarp::os::Bottle* inputPtr = value->asList();
    if (inputPtr == nullptr)
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] The "
                     "list associated to the value named "
                  << key << " is empty." << std::endl;
        return false;
    }

    if (vector.size() != inputPtr->size())
    {
        // If the vector can be resize, let resize it. Otherwise it is a fix-size vector and the
        // dimensions has to be the same of list
        if constexpr (GenericContainer::is_vector<T>::value)
        {
            if (!vector.resizeVector(inputPtr->size()))
            {
                std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] "
                          << "Unable to resize " << type_name<T>()
                          << "List size: "
                          << inputPtr->size() << ". Vector size: " << vector.size() << std::endl;
                return false;
            }
        }
        else if constexpr (is_resizable<T>::value)
            vector.resize(inputPtr->size());
        else
        {
            std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] "
                         "The size of the vector does not match with the size of the list. List "
                         "size: "
                      << inputPtr->size() << ". Vector size: " << vector.size() << std::endl;
            return false;
        }
    }

    for (int i = 0; i < inputPtr->size(); i++)
    {
        if (!(inputPtr->get(i).*YARP_UTILITES_GET_CHECKER_NAME(elementType))())
        {
            std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] "
                         "The element of the list associated to the value named "
                      << key << " is not a " << YARP_UTILITES_GET_ELEMENT_TYPE(elementType) << "."
                      << std::endl;
            return false;
        }

        vector[i] = convertValue<elementType>(inputPtr->get(i));
    }
    return true;
}

template <typename T> void mergeSigVector(yarp::sig::Vector& vector, const T& t)
{
    if constexpr (std::is_arithmetic<T>::value)
        vector.push_back(t);

    else if constexpr (!std::is_arithmetic<T>::value)
    {
        using elementType = typename std::pointer_traits<decltype(t.data())>::element_type;
        static_assert(std::is_convertible<elementType, double>::value,
                      "[BipedalLocomotionControllers::YarpUtilities::mergeSigVector] The element "
                      "contained in the vector cannot be converted in a double");

        if constexpr (is_iterable<T>::value)
        {
            for (auto it = t.begin(); it != t.end(); it++)
                vector.push_back(*it);

        } else if constexpr (has_square_bracket_operator<T>::value)
        {
            for (int i = 0; i < t.size(); i++)
                vector.push_back(t[i]);

        } else
            static_assert(dependent_false<T>::value,
                          "[BipedalLocomotionControllers::YarpUtilities::mergeSigVector] The "
                          "Vector type does not have square bracket operator nor begin()/end() "
                          "methods");
        return;
    }

    else
    {
        static_assert(dependent_false<T>::value,
                      "[BipedalLocomotionControllers::YarpUtilities::mergeSigVector] The type of "
                      "the input element cannot be handled by the function");
    }

    return;
}

template <typename T, typename... Args>
void mergeSigVector(yarp::sig::Vector& vector, const T& t, const Args&... args)
{
    mergeSigVector<T>(vector, t);
    mergeSigVector<Args...>(vector, args...);

    return;
}

template <typename... Args>
void sendVariadicVector(yarp::os::BufferedPort<yarp::sig::Vector>& port, const Args&... args)
{
    yarp::sig::Vector& vector = port.prepare();
    vector.clear();

    mergeSigVector(vector, args...);

    port.write();
}
} // namespace YarpUtilities
} // namespace BipedalLocomotionControllers

// remove the macro
#undef YARP_UTILITES_GET_ELEMENT_TYPE
#undef YARP_UTILITES_CHECK_ELEMENT_SUPPORT
#undef YARP_UTILITES_GET_CHECKER_NAME
