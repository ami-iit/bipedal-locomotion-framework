/**
 * @file ButterworthLowPassFilter.cpp
 * @authors Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <chrono>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Dense>

#include <BipedalLocomotion/ContinuousDynamicalSystem/ButterworthLowPassFilter.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace BipedalLocomotion::ContinuousDynamicalSystem;

TEST_CASE("ButterworthLowPass filter")
{
    using namespace std::chrono_literals;
    constexpr int order = 20;
    constexpr double cutOffFrequency = 10.0;
    constexpr std::chrono::nanoseconds dT = 10ms;

    auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    paramHandler->setParameter("order", order);
    paramHandler->setParameter("cutoff_frequency", cutOffFrequency);
    paramHandler->setParameter("sampling_time", dT);

    Eigen::VectorXd input = Eigen::VectorXd::Zero(2);
    input << 1.0, 1.0;

    constexpr double tolerance = 1e-3;
    constexpr std::chrono::nanoseconds simulationTime = 2s;

    SECTION("Prewrapping disabled")
    {
        ButterworthLowPassFilter butterworthLowPass;
        paramHandler->setParameter("enable_prewrapping", false);
        REQUIRE(butterworthLowPass.initialize(paramHandler));
        REQUIRE(butterworthLowPass.reset(Eigen::Vector2d::Zero()));

        for (std::chrono::nanoseconds t = 0ns; t < simulationTime; t += dT)
        {
            REQUIRE(butterworthLowPass.setInput(input));
            REQUIRE(butterworthLowPass.advance());
        }

        REQUIRE(butterworthLowPass.getOutput().isApprox(input, tolerance));
    }

    SECTION("Prewrapping enabled")
    {
        ButterworthLowPassFilter butterworthLowPass;
        paramHandler->setParameter("enable_prewrapping", true);
        REQUIRE(butterworthLowPass.initialize(paramHandler));
        REQUIRE(butterworthLowPass.reset(Eigen::Vector2d::Zero()));

        for (std::chrono::nanoseconds t = 0ns; t < simulationTime; t += dT)
        {
            REQUIRE(butterworthLowPass.setInput(input));
            REQUIRE(butterworthLowPass.advance());
        }

        REQUIRE(butterworthLowPass.getOutput().isApprox(input, tolerance));
    }
}
