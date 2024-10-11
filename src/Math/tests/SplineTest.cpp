/**
 * @file SplineTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020-2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <array>
#include <chrono>
#include <vector>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/Math/CubicSpline.h>
#include <BipedalLocomotion/Math/LinearSpline.h>
#include <BipedalLocomotion/Math/QuinticSpline.h>
#include <BipedalLocomotion/Math/ZeroOrderSpline.h>

using namespace BipedalLocomotion::Math;

TEST_CASE("Quintic spline")
{
    using namespace std::chrono_literals;

    constexpr std::size_t numberOfPoints = 100;
    constexpr std::chrono::nanoseconds initTime = 320ms;
    constexpr std::chrono::nanoseconds finalTime = 2s + 640s;

    constexpr std::chrono::nanoseconds dT = (finalTime - initTime) / (numberOfPoints - 1);

    constexpr double tolerance = 1e-5;

    std::array<Eigen::Vector4d, 6> coefficients;
    for (auto& coeff : coefficients)
    {
        coeff.setRandom();
    }

    std::vector<Eigen::Vector4d> knots;
    std::vector<std::chrono::nanoseconds> time;
    for (std::size_t i = 0; i < numberOfPoints; i++)
    {
        time.push_back(dT * i + initTime);
        const double t = std::chrono::duration<double>(time.back()).count();
        knots.push_back(coefficients[0] + coefficients[1] * t + coefficients[2] * std::pow(t, 2)
                        + coefficients[3] * std::pow(t, 3) + coefficients[4] * std::pow(t, 4)
                        + coefficients[5] * std::pow(t, 5));
    }

    const double initT = std::chrono::duration<double>(initTime).count();
    const double finalT = std::chrono::duration<double>(finalTime).count();
    Eigen::Vector4d initVelocity
        = coefficients[1] + 2 * coefficients[2] * initT + 3 * coefficients[3] * std::pow(initT, 2)
          + 4 * coefficients[4] * std::pow(initT, 3) + 5 * coefficients[5] * std::pow(initT, 4);

    Eigen::Vector4d initAcceleration = 2 * coefficients[2] + 3 * 2 * coefficients[3] * initT
                                       + 4 * 3 * coefficients[4] * std::pow(initT, 2)
                                       + 5 * 4 * coefficients[5] * std::pow(initT, 3);

    Eigen::Vector4d finalVelocity
        = coefficients[1] + 2 * coefficients[2] * finalT + 3 * coefficients[3] * std::pow(finalT, 2)
          + 4 * coefficients[4] * std::pow(finalT, 3) + 5 * coefficients[5] * std::pow(finalT, 4);

    Eigen::Vector4d finalAcceleration = 2 * coefficients[2] + 3 * 2 * coefficients[3] * finalT
                                        + 4 * 3 * coefficients[4] * std::pow(finalT, 2)
                                        + 5 * 4 * coefficients[5] * std::pow(finalT, 3);

    QuinticSpline<Eigen::Vector4d> spline;
    REQUIRE(spline.setKnots(knots, time));

    REQUIRE(spline.setInitialConditions({initVelocity, initAcceleration}));
    REQUIRE(spline.setFinalConditions({finalVelocity, finalAcceleration}));

    constexpr std::size_t pointsToCheckNumber = 1e3;

    constexpr std::chrono::nanoseconds dTCheckPoints
        = (finalTime - initTime) / (pointsToCheckNumber);

    Eigen::Vector4d expected, position, velocity, acceleration;

    for (std::size_t i = 0; i < pointsToCheckNumber; i++)
    {
        std::chrono::nanoseconds t = dTCheckPoints * i + initTime;
        double tDouble = std::chrono::duration<double>(t).count();

        REQUIRE(spline.evaluatePoint(t, position, velocity, acceleration));

        // check position
        expected = coefficients[0] + coefficients[1] * tDouble
                   + coefficients[2] * std::pow(tDouble, 2) + coefficients[3] * std::pow(tDouble, 3)
                   + coefficients[4] * std::pow(tDouble, 4)
                   + coefficients[5] * std::pow(tDouble, 5);

        REQUIRE(expected.isApprox(position, tolerance));

        // check velocity
        expected = coefficients[1] + 2 * coefficients[2] * tDouble
                   + 3 * coefficients[3] * std::pow(tDouble, 2)
                   + 4 * coefficients[4] * std::pow(tDouble, 3)
                   + 5 * coefficients[5] * std::pow(tDouble, 4);
        REQUIRE(expected.isApprox(velocity, tolerance));

        // check acceleration
        expected = 2 * coefficients[2] + 3 * 2 * coefficients[3] * tDouble
                   + 4 * 3 * coefficients[4] * std::pow(tDouble, 2)
                   + 5 * 4 * coefficients[5] * std::pow(tDouble, 3);

        REQUIRE(expected.isApprox(acceleration, tolerance));
    }

    SECTION("Advance capabilities")
    {
        REQUIRE_FALSE(spline.isOutputValid());
        REQUIRE(spline.setAdvanceTimeStep(dTCheckPoints));

        for (std::size_t i = 0; i < pointsToCheckNumber; i++)
        {
            double t = std::chrono::duration<double>(dTCheckPoints * i + initTime).count();

            // advance the spline
            REQUIRE(spline.advance());
            REQUIRE(spline.isOutputValid());
            const auto& traj = spline.getOutput();

            // check position
            expected = coefficients[0] + coefficients[1] * t + coefficients[2] * std::pow(t, 2)
                       + coefficients[3] * std::pow(t, 3) + coefficients[4] * std::pow(t, 4)
                       + coefficients[5] * std::pow(t, 5);

            REQUIRE(expected.isApprox(traj.position, tolerance));

            // check velocity
            expected = coefficients[1] + 2 * coefficients[2] * t
                       + 3 * coefficients[3] * std::pow(t, 2) + 4 * coefficients[4] * std::pow(t, 3)
                       + 5 * coefficients[5] * std::pow(t, 4);
            REQUIRE(expected.isApprox(traj.velocity, tolerance));

            // check acceleration
            expected = 2 * coefficients[2] + 3 * 2 * coefficients[3] * t
                       + 4 * 3 * coefficients[4] * std::pow(t, 2)
                       + 5 * 4 * coefficients[5] * std::pow(t, 3);
            REQUIRE(expected.isApprox(traj.acceleration, tolerance));
        }
    }

    SECTION("Query from a vector of times")
    {
        std::vector<std::chrono::nanoseconds> timeVector;
        for (std::size_t i = 0; i < pointsToCheckNumber; i++)
        {
            timeVector.push_back(dTCheckPoints * i + initTime);
        }

        std::vector<Eigen::Vector4d> positionVector, velocityVector, accelerationVector;
        REQUIRE(spline.evaluateOrderedPoints(timeVector, //
                                             positionVector,
                                             velocityVector,
                                             accelerationVector));

        for (std::size_t i = 0; i < pointsToCheckNumber; i++)
        {
            double t = std::chrono::duration<double>(dTCheckPoints * i + initTime).count();

            // check position
            expected = coefficients[0] + coefficients[1] * t + coefficients[2] * std::pow(t, 2)
                       + coefficients[3] * std::pow(t, 3) + coefficients[4] * std::pow(t, 4)
                       + coefficients[5] * std::pow(t, 5);

            REQUIRE(expected.isApprox(positionVector[i], tolerance));

            // check velocity
            expected = coefficients[1] + 2 * coefficients[2] * t
                       + 3 * coefficients[3] * std::pow(t, 2) + 4 * coefficients[4] * std::pow(t, 3)
                       + 5 * coefficients[5] * std::pow(t, 4);
            REQUIRE(expected.isApprox(velocityVector[i], tolerance));

            // check acceleration
            expected = 2 * coefficients[2] + 3 * 2 * coefficients[3] * t
                       + 4 * 3 * coefficients[4] * std::pow(t, 2)
                       + 5 * 4 * coefficients[5] * std::pow(t, 3);
            REQUIRE(expected.isApprox(accelerationVector[i], tolerance));
        }
    }
}

TEST_CASE("Cubic spline")
{
    using namespace std::chrono_literals;

    constexpr std::size_t numberOfPoints = 100;
    constexpr std::chrono::nanoseconds initTime = 320ms;
    constexpr std::chrono::nanoseconds finalTime = 2s + 640s;

    constexpr std::chrono::nanoseconds dT = (finalTime - initTime) / (numberOfPoints - 1);

    constexpr double tolerance = 1e-5;

    std::array<Eigen::Vector4d, 6> coefficients;
    for (auto& coeff : coefficients)
    {
        coeff.setRandom();
    }

    std::vector<Eigen::Vector4d> knots;
    std::vector<std::chrono::nanoseconds> time;
    for (std::size_t i = 0; i < numberOfPoints; i++)
    {
        time.push_back(dT * i + initTime);
        const double t = std::chrono::duration<double>(time.back()).count();
        knots.push_back(coefficients[0] + coefficients[1] * t + coefficients[2] * std::pow(t, 2)
                        + coefficients[3] * std::pow(t, 3));
    }

    const double initT = std::chrono::duration<double>(initTime).count();
    const double finalT = std::chrono::duration<double>(finalTime).count();

    Eigen::Vector4d initVelocity
        = coefficients[1] + 2 * coefficients[2] * initT + 3 * coefficients[3] * std::pow(initT, 2);

    Eigen::Vector4d finalVelocity = coefficients[1] + 2 * coefficients[2] * finalT
                                    + 3 * coefficients[3] * std::pow(finalT, 2);

    Eigen::Vector4d initAcceleration = 2 * coefficients[2] + 3 * 2 * coefficients[3] * initT;
    Eigen::Vector4d finalAcceleration = 2 * coefficients[2] + 3 * 2 * coefficients[3] * finalT;

    CubicSpline<Eigen::Vector4d> spline;
    REQUIRE(spline.setKnots(knots, time));

    REQUIRE(spline.setInitialConditions({initVelocity, initAcceleration}));
    REQUIRE(spline.setFinalConditions({finalVelocity, finalAcceleration}));

    constexpr std::size_t pointsToCheckNumber = 1e3;

    constexpr std::chrono::nanoseconds dTCheckPoints = (finalTime - initTime) / pointsToCheckNumber;

    Eigen::Vector4d expected, position, velocity, acceleration;

    for (std::size_t i = 0; i < pointsToCheckNumber; i++)
    {
        std::chrono::nanoseconds t = dTCheckPoints * i + initTime;
        double tSeconds = std::chrono::duration<double>(t).count();

        REQUIRE(spline.evaluatePoint(t, position, velocity, acceleration));

        // check position
        expected = coefficients[0] + coefficients[1] * tSeconds
                   + coefficients[2] * std::pow(tSeconds, 2)
                   + coefficients[3] * std::pow(tSeconds, 3);

        REQUIRE(expected.isApprox(position, tolerance));

        // check velocity
        expected = coefficients[1] + 2 * coefficients[2] * tSeconds
                   + 3 * coefficients[3] * std::pow(tSeconds, 2);
        REQUIRE(expected.isApprox(velocity, tolerance));
    }

    SECTION("Advance capabilities")
    {
        REQUIRE_FALSE(spline.isOutputValid());
        REQUIRE(spline.setAdvanceTimeStep(dTCheckPoints));

        for (std::size_t i = 0; i < pointsToCheckNumber; i++)
        {
            double t = std::chrono::duration<double>(dTCheckPoints * i + initTime).count();

            // advance the spline
            REQUIRE(spline.advance());
            REQUIRE(spline.isOutputValid());
            const auto& traj = spline.getOutput();

            // check position
            expected = coefficients[0] //
                       + coefficients[1] * t //
                       + coefficients[2] * std::pow(t, 2) //
                       + coefficients[3] * std::pow(t, 3);

            REQUIRE(expected.isApprox(traj.position, tolerance));

            // check velocity
            expected = coefficients[1] //
                       + 2 * coefficients[2] * t //
                       + 3 * coefficients[3] * std::pow(t, 2);
            REQUIRE(expected.isApprox(traj.velocity, tolerance));
        }
    }

    SECTION("Query from a vector of times")
    {
        std::vector<std::chrono::nanoseconds> timeVector;
        for (std::size_t i = 0; i < pointsToCheckNumber; i++)
        {
            timeVector.push_back(dTCheckPoints * i + initTime);
        }

        std::vector<Eigen::Vector4d> positionVector, velocityVector, accelerationVector;
        REQUIRE(spline.evaluateOrderedPoints(timeVector, //
                                             positionVector,
                                             velocityVector,
                                             accelerationVector));

        for (std::size_t i = 0; i < pointsToCheckNumber; i++)
        {
            double t = std::chrono::duration<double>(dTCheckPoints * i + initTime).count();

            // check position
            expected = coefficients[0] + coefficients[1] * t + coefficients[2] * std::pow(t, 2)
                       + coefficients[3] * std::pow(t, 3);

            REQUIRE(expected.isApprox(positionVector[i], tolerance));

            // check velocity
            expected = coefficients[1] //
                       + 2 * coefficients[2] * t //
                       + 3 * coefficients[3] * std::pow(t, 2);
            REQUIRE(expected.isApprox(velocityVector[i], tolerance));
        }
    }
}

TEST_CASE("Linear spline")
{
    using namespace std::chrono_literals;

    constexpr std::size_t numberOfPoints = 100;
    constexpr std::chrono::nanoseconds initTime = 320ms;
    constexpr std::chrono::nanoseconds finalTime = 2s + 640s;

    constexpr std::chrono::nanoseconds dT = (finalTime - initTime) / (numberOfPoints - 1);

    constexpr double tolerance = 1e-5;

    std::array<Eigen::Vector4d, 2> coefficients;
    for (auto& coeff : coefficients)
    {
        coeff.setRandom();
    }

    std::vector<Eigen::Vector4d> knots;
    std::vector<std::chrono::nanoseconds> time;
    for (std::size_t i = 0; i < numberOfPoints; i++)
    {
        time.push_back(dT * i + initTime);
        const double t = std::chrono::duration<double>(time.back()).count();
        knots.push_back(coefficients[0] + coefficients[1] * t);
    }

    const double initT = std::chrono::duration<double>(initTime).count();
    const double finalT = std::chrono::duration<double>(finalTime).count();

    Eigen::Vector4d initVelocity = coefficients[1];
    Eigen::Vector4d finalVelocity = coefficients[1];

    Eigen::Vector4d initAcceleration = Eigen::Vector4d::Zero();
    Eigen::Vector4d finalAcceleration = Eigen::Vector4d::Zero();

    LinearSpline<Eigen::Vector4d> spline;
    REQUIRE(spline.setKnots(knots, time));

    REQUIRE(spline.setInitialConditions({initVelocity, initAcceleration}));
    REQUIRE(spline.setFinalConditions({finalVelocity, finalAcceleration}));

    constexpr std::size_t pointsToCheckNumber = 1e3;

    constexpr std::chrono::nanoseconds dTCheckPoints = (finalTime - initTime) / pointsToCheckNumber;

    Eigen::Vector4d expected, position, velocity, acceleration;

    for (std::size_t i = 0; i < pointsToCheckNumber; i++)
    {
        std::chrono::nanoseconds t = dTCheckPoints * i + initTime;
        double tSeconds = std::chrono::duration<double>(t).count();

        REQUIRE(spline.evaluatePoint(t, position, velocity, acceleration));

        // check position
        expected = coefficients[0] + coefficients[1] * tSeconds;

        REQUIRE(expected.isApprox(position, tolerance));
    }

    SECTION("Advance capabilities")
    {
        REQUIRE_FALSE(spline.isOutputValid());
        REQUIRE(spline.setAdvanceTimeStep(dTCheckPoints));

        for (std::size_t i = 0; i < pointsToCheckNumber; i++)
        {
            double t = std::chrono::duration<double>(dTCheckPoints * i + initTime).count();

            // advance the spline
            REQUIRE(spline.advance());
            REQUIRE(spline.isOutputValid());
            const auto& traj = spline.getOutput();

            // check position
            expected = coefficients[0] + coefficients[1] * t;

            REQUIRE(expected.isApprox(traj.position, tolerance));
        }
    }

    SECTION("Query from a vector of times")
    {
        std::vector<std::chrono::nanoseconds> timeVector;
        for (std::size_t i = 0; i < pointsToCheckNumber; i++)
        {
            timeVector.push_back(dTCheckPoints * i + initTime);
        }

        std::vector<Eigen::Vector4d> positionVector, velocityVector, accelerationVector;
        REQUIRE(spline.evaluateOrderedPoints(timeVector, //
                                             positionVector,
                                             velocityVector,
                                             accelerationVector));

        for (std::size_t i = 0; i < pointsToCheckNumber; i++)
        {
            double t = std::chrono::duration<double>(dTCheckPoints * i + initTime).count();

            // check position
            expected = coefficients[0] + coefficients[1] * t;

            REQUIRE(expected.isApprox(positionVector[i], tolerance));

            // check velocity
            expected = coefficients[1];
            REQUIRE(expected.isApprox(velocityVector[i], tolerance));
        }
    }
}

TEST_CASE("Zero Order spline")
{
    using namespace std::chrono_literals;

    constexpr std::size_t numberOfPoints = 100;
    constexpr std::chrono::nanoseconds initTime = 320ms;
    constexpr std::chrono::nanoseconds finalTime = 2s + 640s;

    constexpr double tolerance = 1e-5;

    constexpr std::chrono::nanoseconds dT = (finalTime - initTime) / (numberOfPoints - 1);
    std::array<Eigen::Vector4d, 1> coefficients;
    for (auto& coeff : coefficients)
    {
        coeff.setRandom();
    }

    std::vector<Eigen::Vector4d> knots;
    std::vector<std::chrono::nanoseconds> time;
    for (std::size_t i = 0; i < numberOfPoints; i++)
    {
        time.push_back(dT * i + initTime);
        const double t = std::chrono::duration<double>(time.back()).count();
        knots.push_back(coefficients[0]);
    }

    const double initT = std::chrono::duration<double>(initTime).count();
    const double finalT = std::chrono::duration<double>(finalTime).count();

    Eigen::Vector4d initVelocity = Eigen::Vector4d::Zero();
    Eigen::Vector4d finalVelocity = Eigen::Vector4d::Zero();

    Eigen::Vector4d initAcceleration = Eigen::Vector4d::Zero();
    Eigen::Vector4d finalAcceleration = Eigen::Vector4d::Zero();

    ZeroOrderSpline<Eigen::Vector4d> spline;
    REQUIRE(spline.setKnots(knots, time));

    REQUIRE(spline.setInitialConditions({initVelocity, initAcceleration}));
    REQUIRE(spline.setFinalConditions({finalVelocity, finalAcceleration}));

    constexpr std::size_t pointsToCheckNumber = 1e3;

    constexpr std::chrono::nanoseconds dTCheckPoints = (finalTime - initTime) / pointsToCheckNumber;

    Eigen::Vector4d expected, position, velocity, acceleration;

    for (std::size_t i = 0; i < pointsToCheckNumber; i++)
    {
        std::chrono::nanoseconds t = dTCheckPoints * i + initTime;
        double tSeconds = std::chrono::duration<double>(t).count();

        REQUIRE(spline.evaluatePoint(t, position, velocity, acceleration));

        // check position
        expected = coefficients[0];

        REQUIRE(expected.isApprox(position, tolerance));
    }

    SECTION("Advance capabilities")
    {
        REQUIRE_FALSE(spline.isOutputValid());
        REQUIRE(spline.setAdvanceTimeStep(dTCheckPoints));

        for (std::size_t i = 0; i < pointsToCheckNumber; i++)
        {
            double t = std::chrono::duration<double>(dTCheckPoints * i + initTime).count();

            // advance the spline
            REQUIRE(spline.advance());
            REQUIRE(spline.isOutputValid());
            const auto& traj = spline.getOutput();

            // check position
            expected = coefficients[0];

            REQUIRE(expected.isApprox(traj.position, tolerance));
        }
    }

    SECTION("Query from a vector of times")
    {
        std::vector<std::chrono::nanoseconds> timeVector;
        for (std::size_t i = 0; i < pointsToCheckNumber; i++)
        {
            timeVector.push_back(dTCheckPoints * i + initTime);
        }

        std::vector<Eigen::Vector4d> positionVector, velocityVector, accelerationVector;
        REQUIRE(spline.evaluateOrderedPoints(timeVector, //
                                             positionVector,
                                             velocityVector,
                                             accelerationVector));

        for (std::size_t i = 0; i < pointsToCheckNumber; i++)
        {
            double t = std::chrono::duration<double>(dTCheckPoints * i + initTime).count();

            // check position
            expected = coefficients[0];

            REQUIRE(expected.isApprox(positionVector[i], tolerance));

            // check velocity and acceleration
            REQUIRE(velocityVector[i].isZero(tolerance));

            REQUIRE(accelerationVector[i].isZero(tolerance));
        }
    }
}
