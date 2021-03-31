/**
 * @file QuinticSplineTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <array>
#include <vector>

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotion/Planners/QuinticSpline.h>

using namespace BipedalLocomotion::Planners;


TEST_CASE("Quintic spline")
{
    const std::size_t numberOfPoints = 100;
    constexpr double initTime = 0.32;
    constexpr double finalTime = 2.64;

    constexpr double dT = (finalTime - initTime) / (static_cast<double>(numberOfPoints - 1));
    std::array<Eigen::Vector4d, 6> coefficients;
    for(auto& coeff: coefficients)
    {
        coeff.setRandom();
    }

    std::vector<Eigen::VectorXd> knots;
    std::vector<double> time;
    for (std::size_t i = 0; i < numberOfPoints; i++)
    {
        time.push_back(dT * i + initTime);
        knots.push_back(coefficients[0] + coefficients[1] * time.back()
                        + coefficients[2] * std::pow(time.back(), 2)
                        + coefficients[3] * std::pow(time.back(), 3)
                        + coefficients[4] * std::pow(time.back(), 4)
                        + coefficients[5] * std::pow(time.back(), 5));
    }

    Eigen::Vector4d initVelocity = coefficients[1] + 2 * coefficients[2] * initTime
                                   + 3 * coefficients[3] * std::pow(initTime, 2)
                                   + 4 * coefficients[4] * std::pow(initTime, 3)
                                   + 5 * coefficients[5] * std::pow(initTime, 4);

    Eigen::Vector4d initAcceleration = 2 * coefficients[2] + 3 * 2 * coefficients[3] * initTime
                                       + 4 * 3 * coefficients[4] * std::pow(initTime, 2)
                                       + 5 * 4 * coefficients[5] * std::pow(initTime, 3);

    Eigen::Vector4d finalVelocity = coefficients[1] + 2 * coefficients[2] * finalTime
                                    + 3 * coefficients[3] * std::pow(finalTime, 2)
                                    + 4 * coefficients[4] * std::pow(finalTime, 3)
                                    + 5 * coefficients[5] * std::pow(finalTime, 4);

    Eigen::Vector4d finalAcceleration = 2 * coefficients[2] + 3 * 2 * coefficients[3] * finalTime
                                        + 4 * 3 * coefficients[4] * std::pow(finalTime, 2)
                                        + 5 * 4 * coefficients[5] * std::pow(finalTime, 3);

    QuinticSpline spline;
    REQUIRE(spline.setKnots(knots, time));

    REQUIRE(spline.setInitialConditions(initVelocity, initAcceleration));
    REQUIRE(spline.setFinalConditions(finalVelocity, finalAcceleration));

    constexpr std::size_t pointsToCheckNumber = 1e4;

    constexpr double dTCheckPoints
        = (finalTime - initTime) / (static_cast<double>(pointsToCheckNumber));

    Eigen::Vector4d expected, position, velocity, acceleration;

    for (std::size_t i = 0; i < pointsToCheckNumber; i++)
    {
        double t = dTCheckPoints * i + initTime;

        REQUIRE(spline.evaluatePoint(t, position, velocity, acceleration));

        // check position
        expected = coefficients[0] + coefficients[1] * t + coefficients[2] * std::pow(t, 2)
                   + coefficients[3] * std::pow(t, 3) + coefficients[4] * std::pow(t, 4)
                   + coefficients[5] * std::pow(t, 5);

        REQUIRE(expected.isApprox(position, 1e-5));

        // check velocity
        expected = coefficients[1] + 2 * coefficients[2] * t + 3 * coefficients[3] * std::pow(t, 2)
                   + 4 * coefficients[4] * std::pow(t, 3) + 5 * coefficients[5] * std::pow(t, 4);
        REQUIRE(expected.isApprox(velocity,  1e-5));

        // check acceleration
        expected = 2 * coefficients[2] + 3 * 2 * coefficients[3] * t
                   + 4 * 3 * coefficients[4] * std::pow(t, 2)
                   + 5 * 4 * coefficients[5] * std::pow(t, 3);
        REQUIRE(expected.isApprox(acceleration,  1e-5));
    }

    SECTION("Advance capabilities")
    {
        REQUIRE_FALSE(spline.isOutputValid());
        REQUIRE(spline.setAdvanceTimeStep(dTCheckPoints));

        REQUIRE(spline.isOutputValid());

        for (std::size_t i = 0; i < pointsToCheckNumber; i++)
        {
            double t = dTCheckPoints * i + initTime;
            const auto& traj = spline.getOutput();

            // check position
            expected = coefficients[0] + coefficients[1] * t + coefficients[2] * std::pow(t, 2)
                       + coefficients[3] * std::pow(t, 3) + coefficients[4] * std::pow(t, 4)
                       + coefficients[5] * std::pow(t, 5);

            REQUIRE(expected.isApprox(traj.position, 1e-5));

            // check velocity
            expected = coefficients[1] + 2 * coefficients[2] * t
                       + 3 * coefficients[3] * std::pow(t, 2) + 4 * coefficients[4] * std::pow(t, 3)
                       + 5 * coefficients[5] * std::pow(t, 4);
            REQUIRE(expected.isApprox(traj.velocity, 1e-5));

            // check acceleration
            expected = 2 * coefficients[2] + 3 * 2 * coefficients[3] * t
                       + 4 * 3 * coefficients[4] * std::pow(t, 2)
                       + 5 * 4 * coefficients[5] * std::pow(t, 3);
            REQUIRE(expected.isApprox(traj.acceleration, 1e-5));

            // advance the spline
            REQUIRE(spline.advance());
        }
    }
}
