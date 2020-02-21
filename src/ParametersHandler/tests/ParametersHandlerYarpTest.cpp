/**
 * @file ParametersHandlerYarpTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// std
#include <memory>

// Catch2
#include <catch2/catch.hpp>

// YARP
#include <yarp/os/Property.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>

#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotionControllers/ParametersHandler/YarpImplementation.h>

using namespace BipedalLocomotionControllers::ParametersHandler;

TEST_CASE("Get parameters")
{
    std::vector<int> fibonacciNumbers{1, 1, 2, 3, 5, 8, 13, 21};
    std::vector<std::string> donaldsNephews{"Huey", "Dewey", "Louie"};

    YarpImplementation::unique_ptr parameterHandler = std::make_unique<YarpImplementation>();
    parameterHandler->setParameter("answer_to_the_ultimate_question_of_life", 42);
    parameterHandler->setParameter("pi", 3.14);
    parameterHandler->setParameter("John", "Smith");
    parameterHandler->setParameter("Fibonacci Numbers", fibonacciNumbers);

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
        REQUIRE(element == fibonacciNumbers);
    }

    SECTION("Get Group")
    {
        YarpImplementation::unique_ptr cartoonsGroup = parameterHandler->getGroup("CARTOONS");
        cartoonsGroup->setParameter("Donald's nephews", donaldsNephews);
        std::vector<std::string> element;
        REQUIRE(cartoonsGroup->getParameter("Donald's nephews", element));
        REQUIRE(element == donaldsNephews);
    }

    SECTION("is Empty")
    {
        YarpImplementation::unique_ptr groupHandler = parameterHandler->getGroup("CARTOONS");
        REQUIRE(groupHandler->isEmpty());

        groupHandler->setParameter("Donald's nephews", donaldsNephews);
        REQUIRE_FALSE(groupHandler->isEmpty());
    }


    SECTION("Print content")
    {
        std::cout << "Parameters: " << *parameterHandler << std::endl;
    }

    SECTION("Set from object")
    {
        yarp::os::ResourceFinder rf;
        parameterHandler->set(rf);
        yarp::os::Property property;
        property.put("value", 10);
        parameterHandler->set(property);
        int expected;
        REQUIRE(parameterHandler->getParameter("value", expected));
        REQUIRE(expected == 10);
    }
}
