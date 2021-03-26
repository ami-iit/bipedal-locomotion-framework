/**
 * @file IntegratorLinearSystem.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <memory>

// Catch2
#include <catch2/catch.hpp>

#include <Eigen/Dense>

#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>

using namespace BipedalLocomotion::ContinuousDynamicalSystem;

TEST_CASE("Integrator - Linear system")
{
    constexpr double dT = 0.0001;
    constexpr double tolerance = 1e-3;
    constexpr double simulationTime = 2;

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

    ForwardEuler<LinearTimeInvariantSystem> integrator;
    REQUIRE(integrator.setIntegrationStep(dT));
    integrator.setDynamicalSystem(system);

    for (int i = 0; i < simulationTime / dT; i++)
    {
        const auto& [solution] = integrator.getSolution();

        REQUIRE(solution.isApprox(closeFormSolution(dT * i), tolerance));
        REQUIRE(integrator.integrate(0, dT));
    }
}
