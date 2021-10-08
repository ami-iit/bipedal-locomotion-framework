/**
 * @file ParametersHandlerRosTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// std
#include <memory>

// Catch2
#include <catch2/catch.hpp>

// ROS
#include <ros/ros.h>

#include <BipedalLocomotion/GenericContainer/Vector.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/RosImplementation.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::GenericContainer;

TEST_CASE("Get parameters")
{
    std::vector<int> fibonacciNumbers{1, 1, 2, 3, 5, 8, 13, 21};
    std::vector<std::string> donaldsNephews{"Huey", "Dewey", "Louie"};
    bool flag = true;
    std::vector<bool> flags{true, false, false, true, true, true};

    std::shared_ptr<RosImplementation> originalHandler = std::make_shared<RosImplementation>();
    IParametersHandler::shared_ptr parameterHandler = originalHandler;
    parameterHandler->setParameter("answer_to_the_ultimate_question_of_life", 42);
    parameterHandler->setParameter("pi", 3.14);
    parameterHandler->setParameter("John", "Smith");
    parameterHandler->setParameter("Fibonacci_Numbers", fibonacciNumbers);
    parameterHandler->setParameter("flag", flag);
    parameterHandler->setParameter("flags", flags);

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

    SECTION("Get Bool")
    {
        bool element;
        REQUIRE(parameterHandler->getParameter("flag", element));
        REQUIRE(element == flag);
    }

    SECTION("Get String")
    {
        std::string element;
        REQUIRE(parameterHandler->getParameter("John", element));
        REQUIRE(element == "Smith");
    }

    SECTION("Change String")
    {
        parameterHandler->setParameter("John", "Doe");
        std::string element;
        REQUIRE(parameterHandler->getParameter("John", element));
        REQUIRE(element == "Doe");
    }

    SECTION("Get Vector")
    {
        std::vector<int> element;
        REQUIRE(parameterHandler->getParameter("Fibonacci_Numbers", element));
        REQUIRE(element == fibonacciNumbers);
    }

    SECTION("Get bool Vector")
    {
        std::vector<bool> element;
        REQUIRE(parameterHandler->getParameter("flags", element));
        REQUIRE(element == flags);
    }

    SECTION("Change Vector")
    {
        fibonacciNumbers.push_back(34);
        parameterHandler->setParameter("Fibonacci_Numbers", fibonacciNumbers);
        std::vector<int> element;
        REQUIRE(parameterHandler->getParameter("Fibonacci_Numbers", element));
        REQUIRE(element == fibonacciNumbers);
    }
}
