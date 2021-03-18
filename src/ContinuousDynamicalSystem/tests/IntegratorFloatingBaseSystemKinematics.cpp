/**
 * @file IntegratorFloatingBaseSystemKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <memory>

// Catch2
#include <catch2/catch.hpp>

#include <Eigen/Dense>

#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>

using namespace BipedalLocomotion::ContinuousDynamicalSystem;

TEST_CASE("Integrator - Linear system")
{
    constexpr double dT = 0.0001;
    constexpr double tolerance = 1e-3;
    constexpr double simulationTime = 0.5;

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

    ForwardEuler<FloatingBaseSystemKinematics> integrator;
    integrator.setIntegrationStep(dT);
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
