/**
 * @file ParametesHandlerYarpTest.cpp
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

#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotionControllers/ParametersHandler/YarpImplementation.h>

using namespace BipedalLocomotionControllers::ParametersHandler;

TEST_CASE("Get parameters")
{
    yarp::os::Property property{{"answer_to_the_ultimate_question_of_life", yarp::os::Value(42)},
                                {"pi", yarp::os::Value(3.14)},
                                {"John", yarp::os::Value("Smith")}};


    std::vector<int> fibonacciNumbers{1, 1, 2, 3, 5, 8, 13, 21};

    {
        yarp::os::Value yarpValue;
        auto list = yarpValue.asList();

        for (const auto& v : fibonacciNumbers)
            list->addInt(v);

        property.put("Fibonacci Numbers", yarpValue);
    }


    auto& group = property.addGroup("CARTOONS");
    std::vector<std::string> donaldsNephews{"Huey", "Dewey", "Louie"};
    {
        yarp::os::Value yarpValue;
        auto list = yarpValue.asList();

        for (const auto& v : donaldsNephews)
            list->addString(v);

        group.put("Donald's nephews", yarpValue);
    }

    std::unique_ptr<IParametersHandler<YarpImplementation>> parameterHandler
        = std::make_unique<YarpImplementation>(property);

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
        std::unique_ptr<IParametersHandler<YarpImplementation>> groupHandler
            = parameterHandler->getGroup("CARTOONS");
        std::vector<std::string> element;
        REQUIRE(groupHandler->getParameter("Donald's nephews", element));
        REQUIRE(element == donaldsNephews);
    }
}
