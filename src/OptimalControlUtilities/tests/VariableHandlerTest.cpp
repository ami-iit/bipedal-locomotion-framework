/**
 * @file ControlProblemTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

TEST_CASE("Test Variable Handler")
{
    // Instantiate the handler
    VariableHandler handler;

    handler.addVariable("base_acceleration", 6);
    handler.addVariable("joint_accelerations", 23);

    REQUIRE(handler.getVariable("base_acceleration").offset == 0);
    REQUIRE(handler.getVariable("base_acceleration").size == 6);

    REQUIRE(handler.getVariable("joint_accelerations").offset == 6);
    REQUIRE(handler.getVariable("joint_accelerations").size == 23);

    REQUIRE(handler.getNumberOfVariables() == 23 + 6);

    REQUIRE(!handler.getVariable("center_of_mass").isValid());
}
