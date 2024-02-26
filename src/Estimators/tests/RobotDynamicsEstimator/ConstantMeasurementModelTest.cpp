/**
 * @file ConstantMeasurementModelTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>

#include <yarp/os/ResourceFinder.h>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/ConstantMeasurementModel.h>

using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;

TEST_CASE("Constant Measurement Model")
{
    auto parameterHandler = std::make_shared<StdImplementation>();

    // Create variable handler
    constexpr size_t sizeVariable = 6;
    VariablesHandler variableHandler;
    REQUIRE(variableHandler.addVariable("JOINT_VELOCITIES", sizeVariable));
    REQUIRE(variableHandler.addVariable("MOTOR_TORQUES", sizeVariable));
    REQUIRE(variableHandler.addVariable("FRICTION_TORQUES", sizeVariable));
    REQUIRE(variableHandler.addVariable("r_leg_ft", sizeVariable));
    REQUIRE(variableHandler.addVariable("r_leg_ft_bias", sizeVariable));
    REQUIRE(variableHandler.addVariable("r_foot_front_ft", sizeVariable));

    Eigen::VectorXd state
        = Eigen::VectorXd::Zero(sizeVariable * variableHandler.getNumberOfVariables());

    const std::string name = "r_leg_ft";
    Eigen::VectorXd covariance(6);
    covariance << 1e-7, 1e-2, 5e0, 5e-3, 5e-1, 5e-10;
    const std::string model = "ConstantMeasurementModel";
    const bool useBias = true;

    parameterHandler->clear();
    parameterHandler->setParameter("name", name);
    parameterHandler->setParameter("associated_state", name);
    parameterHandler->setParameter("covariance", covariance);
    parameterHandler->setParameter("initial_covariance", covariance);
    parameterHandler->setParameter("dynamic_model", model);
    parameterHandler->setParameter("sampling_time", 0.01);
    parameterHandler->setParameter("use_bias", useBias);

    ConstantMeasurementModel ft;

    REQUIRE(ft.initialize(parameterHandler, name));
    REQUIRE(ft.finalize(variableHandler));

    Eigen::VectorXd bias(6);
    for (int index = 0; index < 6; index++)
    {
        bias(index) = GENERATE(take(1, random(-100, 100)));
    }

    state.segment(variableHandler.getVariable("r_leg_ft_bias").offset,
                  variableHandler.getVariable("r_leg_ft_bias").size)
        = bias;

    ft.setState(state);
    REQUIRE(ft.update());

    Eigen::VectorXd updatedVariable = ft.getUpdatedVariable();
    Eigen::VectorXd ftPre = state.segment(variableHandler.getVariable("r_leg_ft").offset,
                                          variableHandler.getVariable("r_leg_ft").size);

    REQUIRE((ftPre + bias).isApprox(updatedVariable));
}
