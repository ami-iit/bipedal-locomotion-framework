/**
 * @file ZeroVelocityStateDynamicsTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>

#include <yarp/os/ResourceFinder.h>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/ZeroVelocityStateDynamics.h>

using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;

TEST_CASE("Zero Velocity Dynamics")
{
    auto parameterHandler = std::make_shared<StdImplementation>();

    // Test without bias
    const std::string name = "tau_m";
    Eigen::VectorXd covariance(6);
    covariance << 1e-3, 1e-3, 5e-3, 5e-3, 5e-3, 5e-3;
    const std::string model = "ZeroVelocityStateDynamics";

    parameterHandler->setParameter("name", name);
    parameterHandler->setParameter("covariance", covariance);
    parameterHandler->setParameter("initial_covariance", covariance);
    parameterHandler->setParameter("dynamic_model", model);
    parameterHandler->setParameter("sampling_time", 0.01);

    // Create variable handler
    constexpr size_t sizeVariable = 6;
    VariablesHandler variableHandler;
    REQUIRE(variableHandler.addVariable("ds", sizeVariable));
    REQUIRE(variableHandler.addVariable("tau_m", sizeVariable));
    REQUIRE(variableHandler.addVariable("tau_F", sizeVariable));
    REQUIRE(variableHandler.addVariable("r_leg_ft", sizeVariable));
    REQUIRE(variableHandler.addVariable("r_leg_ft_bias", sizeVariable));
    REQUIRE(variableHandler.addVariable("r_foot_front_ft", sizeVariable));

    ZeroVelocityStateDynamics tau_m;

    REQUIRE(tau_m.initialize(parameterHandler));
    REQUIRE(tau_m.finalize(variableHandler));

    Eigen::VectorXd currentState(sizeVariable);
    for (int index = 0; index < sizeVariable; index++)
    {
        currentState(index) = GENERATE(take(1, random(-100, 100)));
    }

    Eigen::VectorXd state(sizeVariable * variableHandler.getNumberOfVariables());
    state.setZero();
    state.segment(variableHandler.getVariable("tau_m").offset, variableHandler.getVariable("tau_m").size) = currentState;

    tau_m.setState(state);

    REQUIRE(tau_m.update());

    Eigen::VectorXd nextState(covariance.size());
    nextState = tau_m.getUpdatedVariable();

    REQUIRE(currentState == nextState);
}
