/**
 * @file ParametersHandlerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// std
#include <any>
#include <iostream>
#include <memory>
#include <unordered_map>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/GenericContainer/TemplateHelpers.h>
#include <BipedalLocomotion/GenericContainer/Vector.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion;

TEST_CASE("Get parameters")
{
    std::shared_ptr<StdImplementation> originalHandler = std::make_shared<StdImplementation>();
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

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
                                               element));
        REQUIRE(element == std::vector<int>{1, 1, 2, 3, 5, 8, 13, 21});
    }

    SECTION("Set/Get Group")
    {
        StdImplementation::shared_ptr newGroup = std::make_shared<StdImplementation>();
        REQUIRE(parameterHandler->setGroup("CARTOONS", newGroup));
        StdImplementation::shared_ptr groupHandler
            = parameterHandler->getGroup("CARTOONS").lock();
        REQUIRE(groupHandler);
        groupHandler->setParameter("Donald's nephews",
                                   std::vector<std::string>{"Huey", "Dewey", "Louie"});
        std::vector<std::string> element;
        REQUIRE(groupHandler->getParameter("Donald's nephews",
                                           element));
        REQUIRE(element == std::vector<std::string>{"Huey", "Dewey", "Louie"});
    }

    SECTION("is Empty")
    {
        StdImplementation::shared_ptr newGroup = std::make_shared<StdImplementation>();
        REQUIRE(parameterHandler->setGroup("CARTOONS", newGroup));
        StdImplementation::shared_ptr groupHandler
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
        originalHandler->set(object);
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

    SECTION("Clone")
    {
        auto newHandler = parameterHandler->clone();
        REQUIRE_FALSE(newHandler->isEmpty());

        SECTION("Get integer")
        {
            int element;
            REQUIRE(newHandler->getParameter("answer_to_the_ultimate_question_of_life", element));
            REQUIRE(element == 42);
        }

        SECTION("Get Double")
        {
            double element;
            REQUIRE(newHandler->getParameter("pi", element));
            REQUIRE(element == 3.14);
        }

        SECTION("Get String")
        {
            std::string element;
            REQUIRE(newHandler->getParameter("John", element));
            REQUIRE(element == "Smith");
        }

        SECTION("Get Vector")
        {
            std::vector<int> element;
            REQUIRE(newHandler->getParameter("Fibonacci Numbers", element));
            REQUIRE(element == std::vector<int>{1, 1, 2, 3, 5, 8, 13, 21});
        }
    }
}
