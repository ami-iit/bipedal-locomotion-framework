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

#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>

using namespace BipedalLocomotionControllers::ParametersHandler;

// example of parameters handler
class BasicImplementation : public IParametersHandler<BasicImplementation>
{
    std::unordered_map<std::string, std::any> m_map;

public:
    BasicImplementation(const std::unordered_map<std::string, std::any>& map)
        : m_map{map}
    {}

    BasicImplementation() = default;

    template <typename T> void setParameter(const std::string& parameterName, const T& parameter)
    {
        m_map[parameterName] = std::make_any<T>(parameter);
    }

    template <typename T> bool getParameter(const std::string& parameterName, T& parameter) const
    {
        auto parameterAny = m_map.find(parameterName);
        if (parameterAny == m_map.end())
        {
            std::cerr << "[BasicImplementation::getParameter] Parameter named " << parameterName
                      << " not found." << std::endl;
            return false;
        }

        try
        {
            parameter = std::any_cast<T>(parameterAny->second);
        } catch (const std::bad_any_cast& exception)
        {
            std::cerr << "[BasicImplementation::getParameter] The type of the parameter named "
                      << parameterName << " is different from the one expected" << std::endl;
            return false;
        }

        return true;
    }

    BasicImplementation::weak_ptr getGroup(const std::string& name) const
    {
        auto group = m_map.find(name);
        if (group == m_map.end())
            return BasicImplementation::make_shared();

        std::unordered_map<std::string, std::any> map;
        try
        {
            map = std::any_cast<decltype(map)>(group->second);
        } catch (const std::bad_any_cast& exception)
        {
            std::cerr << "[BasicImplementation::getGroup] The element named " << name
                      << " is not a 'std::unordered_map<std::string, std::any>'" << std::endl;
            return BasicImplementation::make_shared();
        }

        return BasicImplementation::make_shared(map);
    }

    void set(const std::unordered_map<std::string, std::any>& object)
    {
        m_map = object;
    }

    std::string toString() const
    {
        std::string key;
        for (const auto& parameters: m_map)
            key += parameters.first + " ";

        return key;
    }

    bool isEmpty() const
    {
        return m_map.size() == 0;
    }

    ~BasicImplementation() = default;
};

TEST_CASE("Get parameters")
{
    BasicImplementation::unique_ptr parameterHandler = BasicImplementation::make_unique();
    parameterHandler->setParameter("answer_to_the_ultimate_question_of_life", 42);
    parameterHandler->setParameter("pi", 3.14);
    parameterHandler->setParameter("Fibonacci Numbers", std::vector<int>{1, 1, 2, 3, 5, 8, 13, 21});
    parameterHandler->setParameter("John", std::string("Smith"));

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
        REQUIRE(parameterHandler->getParameter("Fibonacci Numbers", element));
        REQUIRE(element == std::vector<int>{1, 1, 2, 3, 5, 8, 13, 21});
    }

    SECTION("Get Group")
    {
        BasicImplementation::shared_ptr groupHandler = parameterHandler->getGroup("CARTOONS").lock();
        REQUIRE(groupHandler);
        groupHandler->setParameter("Donald's nephews",
                                   std::vector<std::string>{"Huey", "Dewey", "Louie"});
        std::vector<std::string> element;
        REQUIRE(groupHandler->getParameter("Donald's nephews", element));
        REQUIRE(element == std::vector<std::string>{"Huey", "Dewey", "Louie"});
    }

    SECTION("is Empty")
    {
        BasicImplementation::shared_ptr groupHandler = parameterHandler->getGroup("CARTOONS").lock();
        REQUIRE(groupHandler);
        REQUIRE(groupHandler->isEmpty());

        groupHandler->setParameter("Donald's nephews",
                                   std::vector<std::string>{"Huey", "Dewey", "Louie"});

        REQUIRE_FALSE(groupHandler->isEmpty());
    }

    SECTION("Print content")
    {
        std::cout << "Parameters: " << *parameterHandler << std::endl;
    }

    SECTION("Set from object")
    {
        std::unordered_map<std::string, std::any> object;
        object["value"] = std::make_any<int>(10);
        parameterHandler->set(object);
        int expected;
        REQUIRE(parameterHandler->getParameter("value", expected));
        REQUIRE(expected == 10);
    }
}
