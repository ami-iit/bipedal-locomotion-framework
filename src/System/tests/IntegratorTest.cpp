/**
 * @file RecursiveLeastSquareTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <memory>

// Catch2
#include <catch2/catch.hpp>

#include <Eigen/Dense>

#include <BipedalLocomotion/System/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/System/ForwardEuler.h>
#include <BipedalLocomotion/System/LinearTimeInvariantSystem.h>

using namespace BipedalLocomotion::System;

TEST_CASE("Integrator - Linear system")
{
    constexpr double dT = 0.0001;
    constexpr double tolerance = 1e-3;
    constexpr double simulationTime = 2;

    SECTION("Linear System")
    {
        // Create the linear system
        /**
         *                     _           _       _  _
         *            d       | 0       1   |     | 0  |
         *           -- x  =  |             | x + |    | u
         *           dt       |_ - 2    - 2_|     |_2 _|
         *
         */

        auto system = std::make_shared<LinearTimeInvariantSystem>();
        Eigen::Matrix2d A;
        A << 0, 1, -2, -2;

        Eigen::Vector2d b;
        b << 0, 2;

        REQUIRE(system->setSystemMatrices(A, b));

        // analyze a step response
        Eigen::VectorXd u(1);
        u(0) = 1;

        Eigen::Vector2d x0;
        x0.setZero();

        // the presented dynamical has the following close form solution in case of step response
        auto closeFormSolution = [](const double& t) {
            Eigen::Vector2d sol;
            sol(0) = 1 - std::exp(-t) * (std::cos(t) + std::sin(t));
            sol(1) = 2 * std::exp(-t) * std::sin(t);
            return sol;
        };

        system->setControlInput({u});
        system->setState({x0});

        ForwardEuler<LinearTimeInvariantSystem> integrator(dT);
        integrator.setDynamicalSystem(system);

        for (int i = 0; i < simulationTime / dT; i++)
        {
            const auto& [solution] = integrator.getSolution();

            REQUIRE(solution.isApprox(closeFormSolution(dT * i), tolerance));
            REQUIRE(integrator.integrate(0, dT));
        }
    }

    SECTION("Floating base System Kinematics")
    {

        auto system = std::make_shared<FloatingBaseSystemKinematics>();

        Eigen::Matrix<double, 6, 1> twist;
        twist.setRandom();

        Eigen::VectorXd jointVelocity(20);
        jointVelocity.setRandom();

        Eigen::Matrix3d rotation0;
        rotation0.setIdentity();

        Eigen::Vector3d position0;
        position0.setZero();

        Eigen::VectorXd jointPosition0(20);
        jointPosition0.setZero();

        auto closeFormSolution = [&](const double& t) {
            Eigen::Vector3d pos = position0 + t * twist.head<3>();
            Eigen::Matrix3d rot
                = Eigen::AngleAxisd(twist.tail<3>().norm() * t, twist.tail<3>().normalized())
                  * rotation0;

            Eigen::VectorXd jointPos = jointPosition0 + t * jointVelocity;
            return std::make_tuple(pos, rot, jointPos);
        };

        system->setControlInput({twist, jointVelocity});
        system->setState({position0, rotation0, jointPosition0});

        ForwardEuler<FloatingBaseSystemKinematics> integrator(dT);
        integrator.setDynamicalSystem(system);

        for (int i = 0; i < simulationTime / dT; i++)
        {
            const auto& [basePosition, baseRotation, jointPosition] = integrator.getSolution();

            const auto& [basePositionExact, baseRotationExact, jointPositionExact]
                = closeFormSolution(dT * i);

            REQUIRE(baseRotation.isApprox(baseRotationExact, tolerance));
            REQUIRE(basePosition.isApprox(basePositionExact, tolerance));
            REQUIRE(jointPosition.isApprox(jointPositionExact, tolerance));

            REQUIRE(integrator.integrate(0, dT));
        }
    }
}
