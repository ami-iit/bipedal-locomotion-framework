/**
 * @file VariableFeasibleRegionTaskTest.cpp
 * @authors Roberto Mauceri
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

// BipedalLocomotion
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TSID/VariableFeasibleRegionTask.h>

#include <Eigen/Dense>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;

TEST_CASE("Variable Feasible Region Task")
{
    const std::string variable1Name = "variable_1";
    const std::string variable2Name = "variable_2";
    constexpr int variable2Size = 20;
    const auto variable1ElementNames = std::vector<std::string>{"foo", "bar", "baz"};
    const auto variable1ControlledElementNames = std::vector<std::string>{"foo", "baz"};

    auto parameterHandler1 = std::make_shared<StdImplementation>();
    parameterHandler1->setParameter("variable_name", variable1Name);
    parameterHandler1->setParameter("variable_size", (int)variable1ControlledElementNames.size());
    parameterHandler1->setParameter("elements_name", variable1ControlledElementNames);

    auto parameterHandler2 = std::make_shared<StdImplementation>();
    parameterHandler2->setParameter("variable_name", variable2Name);
    parameterHandler2->setParameter("variable_size", variable2Size);

    VariablesHandler variablesHandler;

    REQUIRE(variablesHandler.addVariable("dummy1", 10));
    REQUIRE(variablesHandler.addVariable(variable1Name, variable1ElementNames));
    REQUIRE(variablesHandler.addVariable("dummy2", 15));
    REQUIRE(variablesHandler.addVariable(variable2Name, variable2Size));

    SECTION("Element subgroup")
    {
        VariableFeasibleRegionTask task;
        REQUIRE_FALSE(task.isValid());
        REQUIRE(task.initialize(parameterHandler1));
        REQUIRE(task.setVariablesHandler(variablesHandler));
        REQUIRE_FALSE(task.isValid());

        const int variableTotalSize = variablesHandler.getNumberOfVariables();
        const int anyDimension = 12;

        // the matrix C can have any number of rows
        Eigen::MatrixXd C(anyDimension, variable1ControlledElementNames.size());
        C.setRandom();

        Eigen::VectorXd l;
        Eigen::VectorXd u;

        // the vectors l and u must have the same size
        l = Eigen::VectorXd::Ones(anyDimension);
        u = Eigen::VectorXd::Ones(anyDimension + 1);
        REQUIRE_FALSE(task.setFeasibleRegion(C, l, u));

        // the elements of the vector l must be less than the elements of the vector u
        l = Eigen::VectorXd::Ones(anyDimension) * 1.5;
        u = Eigen::VectorXd::Ones(anyDimension);
        REQUIRE_FALSE(task.setFeasibleRegion(C, l, u));

        // valid input
        l = Eigen::VectorXd::Ones(anyDimension);
        u = Eigen::VectorXd::Ones(anyDimension) * 2.5;
        REQUIRE(task.setFeasibleRegion(C, l, u));

        Eigen::Ref<const Eigen::MatrixXd> A = task.getA();
        Eigen::Ref<const Eigen::MatrixXd> B = task.getB();

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

        // check the matrix A
        Eigen::MatrixXd expectedA(2 * anyDimension, variableTotalSize);
        Eigen::MatrixXd expectedT(2 * anyDimension, variable1ControlledElementNames.size());
        Eigen::MatrixXd expectedS(variable1ControlledElementNames.size(), variableTotalSize);
        std::size_t index = variablesHandler.getVariable(variable1Name).offset;

        expectedS.setZero();
        expectedS(0, index + 0) = 1.0;
        expectedS(1, index + 2) = 1.0;

        expectedT << C, -C;
        expectedA = expectedT * expectedS;
        REQUIRE(A.isApprox(expectedA));

        // check the vector B
        Eigen::VectorXd expectedB(2 * anyDimension);
        expectedB << u, -l;
        REQUIRE(B.isApprox(expectedB));
    }

    SECTION("Entire variable")
    {
        VariableFeasibleRegionTask task2;
        REQUIRE_FALSE(task2.isValid());
        REQUIRE(task2.initialize(parameterHandler2));
        REQUIRE(task2.setVariablesHandler(variablesHandler));
        REQUIRE_FALSE(task2.isValid());

        const int variable2ElementSize = variablesHandler.getVariable(variable2Name).size;
        const int variableTotalSize = variablesHandler.getNumberOfVariables();
        const int anyDimension = 21;

        // the matrix C can have any number of rows
        Eigen::MatrixXd C(anyDimension, variable2ElementSize);
        C.setRandom();

        Eigen::VectorXd l;
        Eigen::VectorXd u;

        // valid input
        l = Eigen::VectorXd::Ones(anyDimension);
        u = Eigen::VectorXd::Ones(anyDimension) * 2.5;
        REQUIRE(task2.setFeasibleRegion(C, l, u));

        Eigen::Ref<const Eigen::MatrixXd> A = task2.getA();
        Eigen::Ref<const Eigen::VectorXd> B = task2.getB();

        // subA == ExpectedT implies subS == identity
        Eigen::MatrixXd expectedT(2 * anyDimension, variable2ElementSize);
        Eigen::MatrixXd subA;
        int variable2ElementOffset = variablesHandler.getVariable(variable2Name).offset;
        expectedT << C, -C;
        subA = A.block(0, variable2ElementOffset, 2 * anyDimension, variable2ElementSize);
        REQUIRE(subA.isApprox(expectedT));

        // check the vector B
        Eigen::VectorXd expectedB(2 * anyDimension);
        expectedB << u, -l;
        REQUIRE(B.isApprox(expectedB));
    }
}
