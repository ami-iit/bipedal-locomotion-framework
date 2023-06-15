/**
 * @file JointsTrackingTaskTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

// BipedalLocomotion
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TSID/VariableRegularizationTask.h>

#include <Eigen/Dense>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;

TEST_CASE("Variable Regularization task")
{
    const std::string variable1Name = "variable_1";
    const std::string variable2Name = "variable_2";
    constexpr int variable2Size = 20;
    const auto elementNames = std::vector<std::string>{"foo", "baz"};

    auto parameterHandler1 = std::make_shared<StdImplementation>();
    parameterHandler1->setParameter("variable_name", variable1Name);
    parameterHandler1->setParameter("variable_size", (int)elementNames.size());
    parameterHandler1->setParameter("elements_name", elementNames);

    auto parameterHandler2 = std::make_shared<StdImplementation>();
    parameterHandler2->setParameter("variable_name", variable2Name);
    parameterHandler2->setParameter("variable_size", variable2Size);

    VariablesHandler variablesHandler;
    REQUIRE(variablesHandler.addVariable("dummy1", 10));
    REQUIRE(variablesHandler.addVariable(variable1Name, {"foo", "bar", "baz"}));
    REQUIRE(variablesHandler.addVariable("dummy2", 15));
    REQUIRE(variablesHandler.addVariable(variable2Name, variable2Size));

    SECTION("Element subgroup")
    {

        VariableRegularizationTask task;
        REQUIRE_FALSE(task.isValid());
        REQUIRE(task.initialize(parameterHandler1));
        REQUIRE(task.setVariablesHandler(variablesHandler));
        REQUIRE_FALSE(task.isValid());

        Eigen::VectorXd regularize(elementNames.size());
        regularize.setRandom();
        REQUIRE(task.setSetPoint(regularize));

        Eigen::Ref<const Eigen::MatrixXd> A = task.getA();

        // check the matrix A
        REQUIRE(A.middleCols(variablesHandler.getVariable("dummy1").offset,
                             variablesHandler.getVariable("dummy1").size)
                    .isZero());

        REQUIRE(A.middleCols(variablesHandler.getVariable("dummy2").offset,
                             variablesHandler.getVariable("dummy2").size)
                    .isZero());

        REQUIRE(A.middleCols(variablesHandler.getVariable(variable2Name).offset,
                             variablesHandler.getVariable(variable2Name).size)
                    .isZero());

        Eigen::MatrixXd expectedSubA(2, 3);
        expectedSubA << 1, 0, 0,
                        0, 0, 1;

        REQUIRE(A.middleCols(variablesHandler.getVariable(variable1Name).offset,
                             variablesHandler.getVariable(variable1Name).size)
                    .isApprox(expectedSubA));

        // check the vector b
        REQUIRE(task.getB().isApprox(regularize));
    }

    SECTION("Entire variable")
    {
        VariableRegularizationTask task;
        REQUIRE_FALSE(task.isValid());
        REQUIRE(task.initialize(parameterHandler2));
        REQUIRE(task.setVariablesHandler(variablesHandler));
        REQUIRE_FALSE(task.isValid());

        Eigen::VectorXd regularize(variable2Size);
        regularize.setRandom();
        REQUIRE(task.setSetPoint(regularize));

        Eigen::Ref<const Eigen::MatrixXd> A = task.getA();

        // check the matrix A
        REQUIRE(A.middleCols(variablesHandler.getVariable("dummy1").offset,
                             variablesHandler.getVariable("dummy1").size)
                    .isZero());

        REQUIRE(A.middleCols(variablesHandler.getVariable("dummy2").offset,
                             variablesHandler.getVariable("dummy2").size)
                    .isZero());

        REQUIRE(A.middleCols(variablesHandler.getVariable(variable1Name).offset,
                             variablesHandler.getVariable(variable1Name).size)
                    .isZero());

        REQUIRE(A.middleCols(variablesHandler.getVariable(variable2Name).offset,
                             variablesHandler.getVariable(variable2Name).size)
                    .isIdentity());

        // check the vector b
        REQUIRE(task.getB().isApprox(regularize));
    }
}
