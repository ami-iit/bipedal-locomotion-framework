/**
 * @file VariablesHandlerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotion/System/VariablesHandler.h>

using namespace BipedalLocomotion::System;

TEST_CASE("Test Variable Handler")
{
    // Instantiate the handler
    VariablesHandler handler;

    constexpr std::size_t variable1Size = 42;
    constexpr std::size_t variable2Size = 35;

    REQUIRE(handler.addVariable("variable_1", variable1Size));
    REQUIRE(handler.addVariable("variable_2", variable2Size));

    REQUIRE(handler.getVariable("variable_1").offset == 0);
    REQUIRE(handler.getVariable("variable_1").size == variable1Size);

    REQUIRE(handler.getVariable("variable_2").offset == variable1Size);
    REQUIRE(handler.getVariable("variable_2").size == variable2Size);

    REQUIRE(handler.getNumberOfVariables() == variable1Size + variable2Size);

    REQUIRE_FALSE(handler.getVariable("variable_3").isValid());
}
