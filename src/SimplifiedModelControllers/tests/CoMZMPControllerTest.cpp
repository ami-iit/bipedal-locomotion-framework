/**
 * @file CoMZMPControllerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <array>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/SimplifiedModelControllers/CoMZMPController.h>

using namespace BipedalLocomotion::SimplifiedModelControllers;
using namespace BipedalLocomotion::ParametersHandler;

Eigen::Map<const Eigen::Vector2d> toEigen(const std::array<double, 2>& v)
{
    return Eigen::Map<const Eigen::Vector2d>(v.data(), v.size());
};

TEST_CASE("Controller")
{
    // the controller law is
    // u = dx* + k_com (x* - x) - k_zmp (zmp* - zmp)
    // here we chose
    // k_com = diag([1, 2])
    // k_zmp = diag([3, 4])
    // x* = [0.01, 0]
    // zmp* = [0.01, 0]
    // x = [-0.02, 0.03]
    // zmp = [0.04, -0.01]

    std::array<double, 2> k_zmp{3, 4};
    std::array<double, 2> k_com{1, 2};

    CoMZMPController::Input input;
    input.desiredCoMVelocity.setZero();
    input.desiredCoMPosition << 0.01, 0;
    input.desiredZMPPosition << 0.01, 0;
    input.CoMPosition << -0.02, 0.03;
    input.ZMPPosition << 0.04, -0.01;
    input.angle = 0;

    auto handler = std::make_shared<StdImplementation>();
    handler->setParameter("zmp_gain", k_zmp);
    handler->setParameter("com_gain", k_com);

    CoMZMPController controller;
    REQUIRE_FALSE(controller.isOutputValid());
    REQUIRE(controller.initialize(handler));

    controller.setInput(input);
    REQUIRE(controller.advance());
    REQUIRE(controller.isOutputValid());

    Eigen::Vector2d expectedOutput = input.desiredCoMVelocity;
    expectedOutput.noalias()
        += toEigen(k_com).asDiagonal() * (input.desiredCoMPosition - input.CoMPosition);
    expectedOutput.noalias()
        += toEigen(k_zmp).asDiagonal() * (input.ZMPPosition - input.desiredZMPPosition);

    expectedOutput.isApprox(controller.getOutput());
}
