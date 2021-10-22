/**
 * @file JointsTrackingTaskTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

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
    const std::string variableName = "variable_1";
    const auto elementNames = std::vector<std::string>{"foo", "baz"};

    auto parameterHandler = std::make_shared<StdImplementation>();
    parameterHandler->setParameter("variable_name", variableName);
    parameterHandler->setParameter("elements_name", elementNames);

    VariablesHandler variablesHandler;
    REQUIRE(variablesHandler.addVariable("dummy1", 10));
    REQUIRE(variablesHandler.addVariable(variableName, {"foo", "bar", "baz"}));
    REQUIRE(variablesHandler.addVariable("dummy2", 15));

    VariableRegularizationTask task;
    REQUIRE_FALSE(task.isValid());
    REQUIRE(task.initialize(parameterHandler));
    REQUIRE(task.setVariablesHandler(variablesHandler));
    REQUIRE_FALSE(task.isValid());

    Eigen::VectorXd regularize(elementNames.size());
    regularize.setRandom();

    REQUIRE(task.setSetPoint(regularize));

    // get A and b
    Eigen::Ref<const Eigen::MatrixXd> A = task.getA();
    Eigen::Ref<const Eigen::VectorXd> b = task.getB();

    // check the matrix A
    REQUIRE(A.middleCols(variablesHandler.getVariable("dummy1").offset,
                         variablesHandler.getVariable("dummy1").size)
                .isZero());

    REQUIRE(A.middleCols(variablesHandler.getVariable("dummy2").offset,
                         variablesHandler.getVariable("dummy2").size)
                .isZero());

    Eigen::MatrixXd temp = A.middleCols(variablesHandler.getVariable(variableName).offset,
                                        variablesHandler.getVariable(variableName).size);

    Eigen::MatrixXd expectedSubA(2,3);
    expectedSubA << 1, 0, 0,
                    0, 0, 1;

    REQUIRE(temp.isApprox(expectedSubA));

    // check the vector b
    REQUIRE(b.isApprox(regularize));
}
