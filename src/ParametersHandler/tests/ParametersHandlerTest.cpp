/**
 * @file ParametersHandlerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// std
#include <any>
#include <iostream>
#include <memory>
#include <unordered_map>

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotionControllers/GenericContainer/TemplateHelpers.h>
#include <BipedalLocomotionControllers/GenericContainer/Vector.h>
#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>

using namespace BipedalLocomotionControllers::ParametersHandler;
using namespace BipedalLocomotionControllers;

// example of parameters handler
class BasicImplementation : public IParametersHandler
{
    std::unordered_map<std::string, std::any> m_map;

    template <typename T>
    void setParameterPrivate(const std::string& parameterName, const T& parameter)
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

    template <typename T>
    bool getParameterPrivate(const std::string& parameterName, T& parameter) const
    {
        auto parameterAny = m_map.find(parameterName);
        if (parameterAny == m_map.end())
        {
            std::cerr << "[BasicImplementation::getParameterPrivate] Parameter named "
                      << parameterName << " not found." << std::endl;
            return false;
        }

        if constexpr (std::is_scalar<T>::value || is_string<T>::value)
        {
            try
            {
                parameter = std::any_cast<T>(parameterAny->second);
            } catch (const std::bad_any_cast& exception)
            {
                std::cerr << "[BasicImplementation::getParameterPrivate] The type of the parameter "
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
                std::cerr << "[BasicImplementation::getParameterPrivate] The type of the parameter "
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
                        std::cerr << "[BasicImplementation::getParameterPrivate] Unable to resize "
                                  << type_name<T>() << "List size: " << castedParameter.size()
                                  << ". Vector size: " << parameter.size() << std::endl;
                        return false;
                    }
                } else if constexpr (is_resizable<T>::value)
                    parameter.resize(castedParameter.size());
                else
                {
                    std::cerr << "[BasicImplementation::getParameterPrivate] The size of the "
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

    void setParameter(const std::string& parameterName,
                      const GenericContainer::Vector<const int>& parameter) final
    {
        return setParameterPrivate(parameterName, parameter);
    }
    void setParameter(const std::string& parameterName,
                      const GenericContainer::Vector<const double>& parameter) final
    {
        return setParameterPrivate(parameterName, parameter);
    }

    void setParameter(const std::string& parameterName,
                      const GenericContainer::Vector<const std::string>& parameter) final
    {
        return setParameterPrivate(parameterName, parameter);
    }

    bool getParameter(const std::string& parameterName,
                      GenericContainer::Vector<int>& parameter) const final
    {
        return getParameterPrivate(parameterName, parameter);
    }

    bool getParameter(const std::string& parameterName,
                      GenericContainer::Vector<double>& parameter) const final
    {
        return getParameterPrivate(parameterName, parameter);
    }

    bool getParameter(const std::string& parameterName,
                      GenericContainer::Vector<std::string>& parameter) const final
    {
        return getParameterPrivate(parameterName, parameter);
    }

public:
    BasicImplementation(const std::unordered_map<std::string, std::any>& map)
        : m_map{map}
    {
    }

    BasicImplementation() = default;

    bool getParameter(const std::string& parameterName, int& parameter) const final
    {
        return getParameterPrivate(parameterName, parameter);
    }

    bool getParameter(const std::string& parameterName, double& parameter) const final
    {
        return getParameterPrivate(parameterName, parameter);
    }

    bool getParameter(const std::string& parameterName, std::string& parameter) const final
    {
        return getParameterPrivate(parameterName, parameter);
    }

    bool getParameter(const std::string& parameterName, bool& parameter) const final
    {
        return getParameterPrivate(parameterName, parameter);
    }

    bool getParameter(const std::string& parameterName, std::vector<bool>& parameter) const final
    {
        return getParameterPrivate(parameterName, parameter);
    }

    void setParameter(const std::string& parameterName, const int& parameter) final
    {
        return setParameterPrivate(parameterName, parameter);
    }

    void setParameter(const std::string& parameterName, const double& parameter) final
    {
        return setParameterPrivate(parameterName, parameter);
    }

    void setParameter(const std::string& parameterName, const char* parameter) final
    {
        return setParameterPrivate(parameterName, std::string(parameter));
    }

    void setParameter(const std::string& parameterName, const std::string& parameter) final
    {
        return setParameterPrivate(parameterName, parameter);
    }

    void setParameter(const std::string& parameterName, const bool& parameter) final
    {
        return setParameterPrivate(parameterName, parameter);
    }

    void setParameter(const std::string& parameterName, const std::vector<bool>& parameter) final
    {
        return setParameterPrivate(parameterName, parameter);
    }

    bool setGroup(const std::string& name, shared_ptr newGroup)
    {
        auto downcastedPtr = std::dynamic_pointer_cast<BasicImplementation>(newGroup);
        if (downcastedPtr == nullptr)
        {
            std::cerr << "[BasicImplementation::setGroup] Unable to downcast the pointer to "
                         "BasicImplementation."
                      << std::endl;
            return false;
        }

        m_map[name] = std::make_any<shared_ptr>(downcastedPtr);

        return true;
    }

    IParametersHandler::weak_ptr getGroup(const std::string& name) const final
    {
        auto group = m_map.find(name);
        if (group == m_map.end())
            return std::make_shared<BasicImplementation>();

        shared_ptr map;
        try
        {
            map = std::any_cast<shared_ptr>(group->second);
        } catch (const std::bad_any_cast& exception)
        {
            std::cerr << "[BasicImplementation::getGroup] The element named " << name
                      << " is not a 'std::unordered_map<std::string, std::any>'" << std::endl;
            return std::make_shared<BasicImplementation>();
        }

        return map;
    }

    void set(const std::unordered_map<std::string, std::any>& object)
    {
        m_map = object;
    }

    std::string toString() const final
    {
        std::string key;
        for (const auto& parameters : m_map)
            key += parameters.first + " ";

        return key;
    }

    bool isEmpty() const final
    {
        return m_map.size() == 0;
    }

    void clear() final
    {
        m_map.clear();
    }

    ~BasicImplementation() = default;
};

TEST_CASE("Get parameters")
{
    IParametersHandler::unique_ptr parameterHandler = std::make_unique<BasicImplementation>();
    parameterHandler->setParameter("answer_to_the_ultimate_question_of_life", 42);
    parameterHandler->setParameter("pi", 3.14);
    parameterHandler->setParameter("Fibonacci Numbers", std::vector<int>{1, 1, 2, 3, 5, 8, 13, 21});
    parameterHandler->setParameter("John", "Smith");

    SECTION("Get integer")
    {
        int element;
        REQUIRE(parameterHandler->getParameter("answer_to_the_ultimate_question_of_life", element));
        REQUIRE(element == 42);
    }

    SECTION("Get Double")
    {
        double element;
        REQUIRE(parameterHandler->getParameter("pi", element));
        REQUIRE(element == 3.14);
    }

    SECTION("Get String")
    {
        std::string element;
        REQUIRE(parameterHandler->getParameter("John", element));
        REQUIRE(element == "Smith");
    }

    SECTION("Get Vector")
    {
        std::vector<int> element;
        REQUIRE(parameterHandler->getParameter("Fibonacci Numbers",
                                               element,
                                               GenericContainer::VectorResizeMode::Resizable));
        REQUIRE(element == std::vector<int>{1, 1, 2, 3, 5, 8, 13, 21});
    }

    SECTION("Set/Get Group")
    {
        BasicImplementation::shared_ptr newGroup = std::make_shared<BasicImplementation>();
        REQUIRE(parameterHandler->setGroup("CARTOONS", newGroup));
        BasicImplementation::shared_ptr groupHandler
            = parameterHandler->getGroup("CARTOONS").lock();
        REQUIRE(groupHandler);
        groupHandler->setParameter("Donald's nephews",
                                   std::vector<std::string>{"Huey", "Dewey", "Louie"});
        std::vector<std::string> element;
        REQUIRE(groupHandler->getParameter("Donald's nephews",
                                           element,
                                           GenericContainer::VectorResizeMode::Resizable));
        REQUIRE(element == std::vector<std::string>{"Huey", "Dewey", "Louie"});
    }

    SECTION("is Empty")
    {
        BasicImplementation::shared_ptr newGroup = std::make_shared<BasicImplementation>();
        REQUIRE(parameterHandler->setGroup("CARTOONS", newGroup));
        BasicImplementation::shared_ptr groupHandler
            = parameterHandler->getGroup("CARTOONS").lock();
        REQUIRE(groupHandler);
        REQUIRE(groupHandler->isEmpty());

        groupHandler->setParameter("Donald's nephews",
                                   std::vector<std::string>{"Huey", "Dewey", "Louie"});

        REQUIRE_FALSE(groupHandler->isEmpty());
    }

    SECTION("Print content")
    {
        std::cout << "Parameters: " << parameterHandler->toString() << std::endl;
    }

    SECTION("Set from object")
    {
        std::unordered_map<std::string, std::any> object;
        object["value"] = std::make_any<int>(10);
        static_cast<BasicImplementation*>(parameterHandler.get())->set(object);
        int expected;
        REQUIRE(parameterHandler->getParameter("value", expected));
        REQUIRE(expected == 10);
    }

    SECTION("Clear")
    {
        REQUIRE_FALSE(parameterHandler->isEmpty());
        parameterHandler->clear();
        REQUIRE(parameterHandler->isEmpty());
    }
}
