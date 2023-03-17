/**
 * @file SO3PlannerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <Eigen/Geometry> // Required because of https://github.com/artivis/manif/issues/162

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotion/Planners/SO3Planner.h>

#include <manif/SO3.h>

using namespace BipedalLocomotion::Planners;

TEST_CASE("SO3 planner")
{
    using namespace std::chrono_literals;

    manif::SO3d initialTranform = manif::SO3d::Random();
    manif::SO3d finalTranform = manif::SO3d::Random();

    constexpr std::chrono::nanoseconds T = 1s;
    constexpr std::chrono::nanoseconds dT = 100us;
    constexpr std::size_t samples = T / dT;

    constexpr double tolerance = 1e-3;

    SECTION("Left - Trivialized [Body]")
    {
        SO3PlannerBody planner;
        REQUIRE(planner.setRotations(initialTranform, finalTranform, T));

        manif::SO3d rotation, predictedRotation;
        manif::SO3d::Tangent velocity, predictedVelocity;
        manif::SO3d::Tangent acceleration;
        REQUIRE(planner.evaluatePoint(0s, rotation, velocity, acceleration));
        predictedRotation = rotation;
        predictedVelocity = velocity;

        REQUIRE(rotation.isApprox(initialTranform, tolerance));
        REQUIRE(velocity.isApprox(manif::SO3d::Tangent::Zero(), tolerance));
        REQUIRE(acceleration.isApprox(manif::SO3d::Tangent::Zero(), tolerance));

        for (std::size_t i = 1; i < samples; i++)
        {
            // propagate the system
            predictedRotation = rotation + (velocity * std::chrono::duration<double>(dT).count());
            predictedVelocity = velocity + (acceleration * std::chrono::duration<double>(dT).count());

            planner.evaluatePoint(i * dT, rotation, velocity, acceleration);

            REQUIRE(predictedRotation.isApprox(rotation, tolerance));
            REQUIRE(predictedVelocity.isApprox(velocity, tolerance));
        }

        planner.evaluatePoint(T, rotation, velocity, acceleration);
        REQUIRE(rotation.isApprox(finalTranform, tolerance));
        REQUIRE(velocity.isApprox(manif::SO3d::Tangent::Zero(), tolerance));
        REQUIRE(acceleration.isApprox(manif::SO3d::Tangent::Zero(), tolerance));
    }

    SECTION("Right - Trivialized [Inertial]")
    {
        SO3PlannerInertial planner;
        REQUIRE(planner.setRotations(initialTranform, finalTranform, T));

        manif::SO3d rotation, predictedRotation;
        manif::SO3d::Tangent velocity, predictedVelocity;
        manif::SO3d::Tangent acceleration;
        REQUIRE(planner.evaluatePoint(0s, rotation, velocity, acceleration));
        predictedRotation = rotation;
        predictedVelocity = velocity;

        REQUIRE(rotation.isApprox(initialTranform, tolerance));
        REQUIRE(velocity.isApprox(manif::SO3d::Tangent::Zero(), tolerance));
        REQUIRE(acceleration.isApprox(manif::SO3d::Tangent::Zero(), tolerance));

        for (std::size_t i = 1; i < samples; i++)
        {
            // propagate the system
            predictedRotation = (velocity * std::chrono::duration<double>(dT).count()) + rotation;
            predictedVelocity = velocity + (acceleration * std::chrono::duration<double>(dT).count());

            planner.evaluatePoint(i * dT, rotation, velocity, acceleration);

            REQUIRE(predictedRotation.isApprox(rotation, tolerance));
            REQUIRE(predictedVelocity.isApprox(velocity, tolerance));
        }

        planner.evaluatePoint(T, rotation, velocity, acceleration);
        REQUIRE(rotation.isApprox(finalTranform, tolerance));
        REQUIRE(velocity.isApprox(manif::SO3d::Tangent::Zero(), tolerance));
        REQUIRE(acceleration.isApprox(manif::SO3d::Tangent::Zero(), tolerance));
    }
}
