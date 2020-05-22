/**
 * @file RecursiveLeastSquareTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <memory>
#include <iostream>

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotion/System/ForwardEuler.h>
#include <BipedalLocomotion/System/LinearTimeInvariantSystem.h>

using namespace BipedalLocomotion::System;

template <typename T, typename U>
bool areVectorsEqual(const T& vector1, const U& vector2, const double& tollerance = 0)
{
    // the tollerance must be a positive number
    if (tollerance < 0)
        return false;

    // check the size of the two vectors
    if (vector1.size() != vector2.size())
        return false;

    // iterate over all the elements
    for (unsigned int i = 0; i < vector1.size(); i++)
        if (std::abs(vector1[i] - vector2[i]) > tollerance)
            return false;

    return true;
}

TEST_CASE("Integrate - Linear system")
{
    constexpr double dT = 0.001;

    // Create the linear system
    /**
     *                     _           _       _  _
     *            d       | 0       1   |     | 0  |
     *           -- x  =  |             | x + |    | u
     *           dt       |_ - 2    - 2_|     |_2 _|
     *
     */

    auto system = std::make_shared<LinearTimeInvariantSystem>();
    iDynTree::MatrixDynSize A(2,2);
    A(0,0) = 0;
    A(0,1) = 1;
    A(1,0) = -2;
    A(1,1) = -2;

    iDynTree::MatrixDynSize B(2,1);
    B(0,0) = 0;
    B(1,0) = 2;

    REQUIRE(system->setSystemMatrices(A, B));

    // analyze a step response
    iDynTree::VectorDynSize u(1);
    u(0) = 1;

    iDynTree::VectorDynSize x0(2);
    x0.zero();

    // the presented dynamical has the following close form solution in case of step response
    auto closeFormSolution = [](const double& t) {
        iDynTree::Vector2 sol;
        sol(0) = 1 - std::exp(-t) * (std::cos(t) + std::sin(t));
        sol(1) = 2 * std::exp(-t) * std::sin(t);
        return sol;
    };

    system->setControlInput({u});
    system->setInitialState({x0});

    ForwardEuler<LinearTimeInvariantSystem> integrator(dT);
    integrator.setDynamicalSystem(system);

    constexpr double tollerance = 1e-3;
    constexpr double simulationTime = 10;
    for (int i = 0; i < simulationTime / dT; i++)
    {
        auto [solution] = integrator.getSolution();
        REQUIRE(areVectorsEqual(solution, closeFormSolution(dT * i), tollerance));

        REQUIRE(integrator.integrate(0, dT));
    }
}
