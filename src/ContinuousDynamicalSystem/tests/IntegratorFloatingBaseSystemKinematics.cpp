/**
 * @file IntegratorFloatingBaseSystemKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <memory>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Dense>

#include <manif/SO3.h>

#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/TestUtils/MemoryAllocationMonitor.h>

using namespace BipedalLocomotion::ContinuousDynamicalSystem;
using namespace BipedalLocomotion::TestUtils;

TEST_CASE("Integrator - Linear system")
{
    using namespace std::chrono_literals;
    constexpr std::chrono::nanoseconds dT = 100us;
    constexpr double tolerance = 1e-3;
    constexpr std::chrono::nanoseconds simulationTime = 500ms;

    auto system = std::make_shared<FloatingBaseSystemKinematics>();

    Eigen::Matrix<double, 6, 1> twist;
    twist.setRandom();

    Eigen::VectorXd jointVelocity(20);
    jointVelocity.setRandom();

    manif::SO3d rotation0;
    rotation0.setIdentity();

    Eigen::Vector3d position0;
    position0.setZero();

    Eigen::VectorXd jointPosition0(20);
    jointPosition0.setZero();

    auto closeFormSolution = [&](const double& t) {
        Eigen::Vector3d pos = position0 + t * twist.head<3>();
        Eigen::Matrix3d rot
            = Eigen::AngleAxisd(twist.tail<3>().norm() * t, twist.tail<3>().normalized())
            * rotation0.rotation();

        Eigen::VectorXd jointPos = jointPosition0 + t * jointVelocity;
        return std::make_tuple(pos, rot, jointPos);
    };

    system->setState({position0, rotation0, jointPosition0});

    ForwardEuler<FloatingBaseSystemKinematics> integrator;
    integrator.setIntegrationStep(dT);
    integrator.setDynamicalSystem(system);

    system->setControlInput({twist, jointVelocity});

    for (int i = 0; i < simulationTime / dT; i++)
    {
        const auto& [basePosition, baseRotation, jointPosition] = integrator.getSolution();

        const auto& [basePositionExact, baseRotationExact, jointPositionExact]
            = closeFormSolution(std::chrono::duration<double>(dT * i).count());

        REQUIRE(baseRotation.rotation().isApprox(baseRotationExact, tolerance));
        REQUIRE(basePosition.isApprox(basePositionExact, tolerance));
        REQUIRE(jointPosition.isApprox(jointPositionExact, tolerance));

        if (i >= 1)
        {
            MemoryAllocationMonitor::startMonitor();
        }
        REQUIRE(integrator.integrate(0s, dT));
        if (i >= 1)
        {
            REQUIRE(MemoryAllocationMonitor::endMonitorAndCheckNoMemoryAllocationInLastMonitor());
        }
    }
}
