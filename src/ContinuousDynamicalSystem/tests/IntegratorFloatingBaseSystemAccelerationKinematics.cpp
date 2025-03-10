/**
 * @file IntegratorFloatingBaseSystemAccelerationKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <memory>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Dense>

#include <manif/SO3.h>

#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemAccelerationKinematics.h>
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
    constexpr std::size_t joints = 8;

    auto system = std::make_shared<FloatingBaseSystemAccelerationKinematics>();

    Eigen::Matrix<double, 6, 1> dtwist;
    dtwist.setRandom();

    Eigen::VectorXd jointAcceleration(joints);
    jointAcceleration.setRandom();

    manif::SO3d rotation0;
    rotation0.setIdentity();

    Eigen::Vector3d position0;
    position0.setRandom();

    std::cerr << "position0: " << position0.transpose() << std::endl;

    Eigen::Vector3d linearVelocity0;
    linearVelocity0.setRandom();

    manif::SO3d::Tangent angularVelocity0;
    angularVelocity0.setZero();

    Eigen::VectorXd jointPosition0(joints);
    jointPosition0.setRandom();

    Eigen::VectorXd jointVelocity0(joints);
    jointVelocity0.setRandom();

    auto closeFormSolution = [&](const double& t) {
        // Equation of the motion in case of constant acceleration
        Eigen::Vector3d pos = position0 + linearVelocity0 * t + dtwist.head<3>() * t * t * 0.5;
        Eigen::Vector3d linearVelocity = linearVelocity0 + t * dtwist.head<3>();

        manif::SO3d::Tangent tmp = angularVelocity0 * t + dtwist.tail<3>() * t * t * 0.5;
        manif::SO3d rot = tmp.exp() * rotation0;
        manif::SO3d::Tangent angularVelocity = angularVelocity0 + t * dtwist.tail<3>();

        Eigen::VectorXd jointPos
            = jointPosition0 + t * jointVelocity0 + t * t * jointAcceleration * 0.5;
        Eigen::VectorXd jointVelocity = jointVelocity0 + t * jointAcceleration;
        return std::make_tuple(pos,
                               rot.rotation(),
                               jointPos,
                               linearVelocity,
                               angularVelocity,
                               jointVelocity);
    };

    system->setState(
        {position0, rotation0, jointPosition0, linearVelocity0, angularVelocity0, jointVelocity0});

    ForwardEuler<FloatingBaseSystemAccelerationKinematics> integrator;
    integrator.setIntegrationStep(dT);
    integrator.setDynamicalSystem(system);

    system->setControlInput({dtwist, jointAcceleration});

    for (int i = 0; i < simulationTime / dT; i++)
    {
        const auto& [basePosition,
                     baseRotation,
                     jointPosition,
                     baseLinearVelocity,
                     baseAngularVelocity,
                     jointVelocity]
            = system->getState();

        const auto& [basePositionExact,
                     baseRotationExact,
                     jointPositionExact,
                     baseLinearVelocityExact,
                     baseAngularVelocityExact,
                     jointVelocityExact]
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
