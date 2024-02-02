/**
 * @file ZeroVelocityStateDynamicsTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>
#include <chrono>

#include <yarp/os/ResourceFinder.h>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/ExternalContactStateDynamics.h>

using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;

TEST_CASE("External Contact State Dynamics")
{
    auto parameterHandler = std::make_shared<StdImplementation>();

    // Test without bias
    const std::string name = "r_sole";
    Eigen::VectorXd covariance(6);
    covariance << 1e-3, 1e-3, 5e-3, 5e-3, 5e-3, 5e-3;
    const std::string model = "ExternalContactStateDynamics";
    Eigen::VectorXd k(6);
    k << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1;
    using namespace std::chrono_literals;
    constexpr std::chrono::nanoseconds dT = 10ms;

    parameterHandler->setParameter("name", name);
    parameterHandler->setParameter("covariance", covariance);
    parameterHandler->setParameter("initial_covariance", covariance);
    parameterHandler->setParameter("dynamic_model", model);
    parameterHandler->setParameter("sampling_time", dT);
    parameterHandler->setParameter("k", k);

    // Create variable handler
    constexpr size_t sizeVariable = 6;
    VariablesHandler variableHandler;
    REQUIRE(variableHandler.addVariable("JOINT_VELOCITIES", sizeVariable));
    REQUIRE(variableHandler.addVariable("MOTOR_TORQUES", sizeVariable));
    REQUIRE(variableHandler.addVariable("FRICTION_TORQUES", sizeVariable));
    REQUIRE(variableHandler.addVariable("r_sole", 6));

    ExternalContactStateDynamics r_sole_contact;

    REQUIRE(r_sole_contact.initialize(parameterHandler, "r_sole"));
    REQUIRE(r_sole_contact.finalize(variableHandler));

    Eigen::VectorXd currentState(covariance.size());
    for (int index = 0; index < covariance.size(); index++)
    {
        currentState(index) = GENERATE(take(1, random(-100, 100)));
    }

    Eigen::VectorXd state(sizeVariable * variableHandler.getNumberOfVariables());
    state.setZero();
    state.segment(variableHandler.getVariable("r_sole").offset,
                  variableHandler.getVariable("r_sole").size)
        = currentState;

    r_sole_contact.setState(state);

    REQUIRE(r_sole_contact.update());

    Eigen::VectorXd nextState(covariance.size());
    nextState = r_sole_contact.getUpdatedVariable();

    Eigen::VectorXd expectedNextState(covariance.size());
    expectedNextState
        = currentState.array()
          - k.array() * currentState.array() * std::chrono::duration<double>(dT).count();

    REQUIRE((expectedNextState - nextState).isZero());
}
