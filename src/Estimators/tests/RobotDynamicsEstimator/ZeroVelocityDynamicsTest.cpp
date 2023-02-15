/**
 * @file ZeroVelocityDynamics.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch.hpp>

#include <ConfigFolderPath.h>
#include <iCubModels/iCubModels.h>
#include <yarp/os/ResourceFinder.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/ZeroVelocityDynamics.h>

using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;

TEST_CASE("Zero Velocity Dynamics")
{
    auto parameterHandler = std::make_shared<StdImplementation>();

    // Test without bias
    const std::string name0 = "tau_m";
    Eigen::VectorXd covariance0(6);
    covariance0 << 1e-3, 1e-3, 5e-3, 5e-3, 5e-3, 5e-3;
    const std::string model0 = "ZeroVelocityDynamics";

    parameterHandler->setParameter("name", name0);
    parameterHandler->setParameter("covariance", covariance0);
    parameterHandler->setParameter("dynamic_model", model0);
    parameterHandler->setParameter("sampling_time", 0.01);

    // Create variable handler
    constexpr size_t sizeVariable = 6;
    VariablesHandler variableHandler;
    REQUIRE(variableHandler.addVariable("ds", sizeVariable));
    REQUIRE(variableHandler.addVariable("tau_m", sizeVariable));
    REQUIRE(variableHandler.addVariable("tau_F", sizeVariable));
    REQUIRE(variableHandler.addVariable("r_leg_ft_sensor", sizeVariable));
    REQUIRE(variableHandler.addVariable("r_leg_ft_sensor_bias", sizeVariable));
    REQUIRE(variableHandler.addVariable("r_foot_front_ft_sensor", sizeVariable));

    ZeroVelocityDynamics tau_m;

    REQUIRE(tau_m.initialize(parameterHandler));
    REQUIRE(tau_m.finalize(variableHandler));

    Eigen::VectorXd currentState(sizeVariable);
    for (int index = 0; index < sizeVariable; index++)
    {
        currentState(index) = GENERATE(take(1, random(-100, 100)));
    }

    Eigen::VectorXd state;
    state.resize(sizeVariable * variableHandler.getNumberOfVariables());
    state.setZero();
    state.segment(variableHandler.getVariable("tau_m").offset, variableHandler.getVariable("tau_m").size) = currentState;

    tau_m.setState(state);

    REQUIRE(tau_m.update());

    Eigen::VectorXd nextState(covariance0.size());
    nextState = tau_m.getUpdatedVariable();

    REQUIRE(currentState == nextState);

    // Test with bias
    const std::string name1 = "r_leg_ft_sensor";
    Eigen::VectorXd covariance1(6);
    covariance1 << 1e-7, 1e-2, 5e0, 5e-3, 5e-1, 5e-10;
    const std::string model1 = "ZeroVelocityDynamics";
    const bool useBias = true;

    parameterHandler->clear();
    parameterHandler->setParameter("name", name1);
    parameterHandler->setParameter("covariance", covariance1);
    parameterHandler->setParameter("dynamic_model", model1);
    parameterHandler->setParameter("sampling_time", 0.01);
    parameterHandler->setParameter("use_bias", useBias);

    ZeroVelocityDynamics ft;

    REQUIRE(ft.initialize(parameterHandler));
    REQUIRE(ft.finalize(variableHandler));

    state.segment(variableHandler.getVariable("r_leg_ft_sensor_bias").offset, variableHandler.getVariable("r_leg_ft_sensor_bias").size) = currentState;

    ft.setState(state);
    REQUIRE(ft.update());

    Eigen::VectorXd updatedVariable = ft.getUpdatedVariable();
    Eigen::VectorXd ftPre = state.segment(variableHandler.getVariable("r_leg_ft_sensor").offset, variableHandler.getVariable("r_leg_ft_sensor").size);
    REQUIRE(std::abs(((currentState.array() + ftPre.array()) - updatedVariable.array()).array().sum()) == 0.0);
}
