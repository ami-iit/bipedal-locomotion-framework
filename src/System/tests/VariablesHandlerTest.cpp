/**
 * @file VariablesHandlerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2019,2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified
 * and distributed under the terms of the GNU Lesser General Public License v2.1 or any later
 * version.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

using namespace BipedalLocomotion::System;

TEST_CASE("Test Variable Handler")
{
    // Instantiate the handler
    VariablesHandler handler;

    SECTION("Add variables - manually")
    {
        constexpr std::size_t variable1Size = 42;
        constexpr std::size_t variable2Size = 35;
        constexpr std::size_t variable3Size = 3;

        REQUIRE(handler.addVariable("variable_1", variable1Size));
        REQUIRE(handler.addVariable("variable_2", variable2Size));

        REQUIRE(handler.getVariable("variable_1").offset == 0);
        REQUIRE(handler.getVariable("variable_1").size == variable1Size);

        REQUIRE(handler.getVariable("variable_2").offset == variable1Size);
        REQUIRE(handler.getVariable("variable_2").size == variable2Size);

        REQUIRE(handler.getNumberOfVariables() == variable1Size + variable2Size);
        REQUIRE_FALSE(handler.getVariable("variable_3").isValid());

        auto variable2 = handler.getVariable("variable_2");
        REQUIRE(variable2.getElementIndex(10) == 10 + variable1Size);
        REQUIRE(variable2.getElementIndex("variable_2_10") == 10 + variable1Size);

        REQUIRE(handler.addVariable("variable_3", variable3Size, {"foo", "bar", "ok"}));
        auto variable3 = handler.getVariable("variable_3");
        REQUIRE(variable3.getElementIndex("bar") == variable1Size + variable2Size + 1);
    }

    SECTION("Add variables - ParametersHandler")
    {
        using namespace BipedalLocomotion::ParametersHandler;
        auto paramHandler = std::make_shared<StdImplementation>();

        std::vector<int> variablesSize{10, 23, 4, 12, 131, 2, 263};
        std::vector<std::string> variablesName{"variable_1",
                                               "variable_2",
                                               "variable_3",
                                               "variable_4",
                                               "variable_5",
                                               "variable_6"};

        paramHandler->setParameter("variables_name", variablesName);
        paramHandler->setParameter("variables_size", variablesSize);

        REQUIRE_FALSE(handler.initialize(paramHandler));

        variablesName.emplace_back("variable_7");
        paramHandler->setParameter("variables_name", variablesName);

        REQUIRE(handler.initialize(paramHandler));

        int offset = 0;
        for (int i = 0; i < variablesSize.size(); i++)
        {
            REQUIRE(handler.getVariable(variablesName[i]).offset == offset);
            REQUIRE(handler.getVariable(variablesName[i]).size == variablesSize[i]);
            offset += handler.getVariable(variablesName[i]).size;
        }
    }
}
