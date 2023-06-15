/**
 * @file FrictionConesTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/Math/ContactWrenchCone.h>
#include <BipedalLocomotion/Math/LinearizedFrictionCone.h>
#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace BipedalLocomotion::Math;
using namespace BipedalLocomotion::ParametersHandler;

TEST_CASE("Linearized Friction Cone")
{
    auto params = std::make_shared<StdImplementation>();
    params->setParameter("number_of_slices", 2);
    params->setParameter("static_friction_coefficient", 0.3);

    LinearizedFrictionCone cone;
    REQUIRE(cone.initialize(params));

    // test the solution
    REQUIRE(cone.getB().isApprox(Eigen::VectorXd::Zero(8)));
    Eigen::MatrixXd matlabSolution(8, 3);
    matlabSolution << 2.4142,  1.0000, -0.7243,
                      0.4142,  1.0000, -0.3000,
                     -0.4142,  1.0000, -0.3000,
                     -2.4142,  1.0000, -0.7243,
                     -2.4142, -1.0000, -0.7243,
                     -0.4142, -1.0000, -0.3000,
                      0.4142, -1.0000, -0.3000,
                      2.4142, -1.0000, -0.7243;

    constexpr double tolerance = 1e-4;
    REQUIRE(matlabSolution.isApprox(cone.getA(), tolerance));
}


TEST_CASE("Contact wrench Cone")
{
    auto params = std::make_shared<StdImplementation>();
    std::vector<double>limitsX{-0.05, 0.1};
    std::vector<double>limitsY{-0.02, 0.01};
    params->setParameter("number_of_slices", 2);
    params->setParameter("static_friction_coefficient", 0.3);
    params->setParameter("foot_limits_x", limitsX);
    params->setParameter("foot_limits_y", limitsY);

    ContactWrenchCone cone;
    REQUIRE(cone.initialize(params));

    SECTION("Check matrix and vector")
    {
        // test the solution
        REQUIRE(cone.getB().isApprox(Eigen::VectorXd::Zero(8 + 12)));

        Eigen::MatrixXd forceConstraintSolution(8, 3);
        forceConstraintSolution << 2.4142,  1.0000, -0.7243,
                                   0.4142,  1.0000, -0.3000,
                                  -0.4142,  1.0000, -0.3000,
                                  -2.4142,  1.0000, -0.7243,
                                  -2.4142, -1.0000, -0.7243,
                                  -0.4142, -1.0000, -0.3000,
                                   0.4142, -1.0000, -0.3000,
                                   2.4142, -1.0000, -0.7243;

        constexpr double tolerance = 1e-4;
        REQUIRE(forceConstraintSolution.isApprox(cone.getA().topLeftCorner<8, 3>(), tolerance));
        REQUIRE(cone.getA().topRightCorner<8, 3>().isZero(tolerance));

        Eigen::MatrixXd frictionConeConstraint(4, 6);
        frictionConeConstraint << 0, 0,  limitsY[0], -1,  0, 0,
                                  0, 0, -limitsY[1],  1,  0, 0,
                                  0, 0,  limitsX[0],  0,  1, 0,
                                  0, 0, -limitsX[1],  0, -1, 0;

        REQUIRE(cone.getA().middleRows(8, 4).isApprox(frictionConeConstraint, tolerance));

        Eigen::MatrixXd yawTorqueConstriantMatlab(8,6);
        yawTorqueConstriantMatlab << -0.01, -0.05, -0.018,  0.300,  0.300, -1.0000,
                                     -0.01,  0.10, -0.033,  0.300, -0.300, -1.0000,
                                      0.02, -0.05, -0.021, -0.300,  0.300, -1.0000,
                                      0.02,  0.10, -0.036, -0.300, -0.300, -1.0000,
                                      0.01,  0.05, -0.018,  0.300,  0.300,  1.0000,
                                      0.01, -0.10, -0.033,  0.300, -0.300,  1.0000,
                                     -0.02,  0.05, -0.021, -0.300,  0.300,  1.0000,
                                     -0.02, -0.10, -0.036, -0.300, -0.300,  1.0000;

        REQUIRE(cone.getA().bottomRows(8).isApprox(yawTorqueConstriantMatlab, tolerance));
    }

    SECTION("Feasible wrench")
    {
        constexpr double forceZ = 100; // normal force in Newton

        // the CoP is feasible since it is inside the rectangular foot.
        Eigen::Vector2d CoP;
        CoP << 0.01, -0.01;
        constexpr double scalingTangentialForce = 0.01;

        const double forceX = forceZ * scalingTangentialForce;
        const double forceY = -forceZ * scalingTangentialForce;
        const double torqueX = forceZ * CoP(1);
        const double torqueY = -forceZ * CoP(0);
        const double torqueZ = 0;

        BipedalLocomotion::Math::Wrenchd wrench;
        wrench << forceX, forceY, forceZ, torqueX, torqueY, torqueZ;

        // Check feasibility
        // Ax - b <= 0
        Eigen::VectorXd constraintValue = -cone.getB();
        constraintValue.noalias() += cone.getA() * wrench;
        REQUIRE((constraintValue.array() < 0.0).all());
    }

    SECTION("Unfeasible wrench")
    {
        constexpr double forceZ = 100; // normal force in Newton

        // The following CoP is outside the rectangular foot. So it is unfeasible.
        Eigen::Vector2d CoP;
        CoP << 0.2, -0.4;

        // The following
        constexpr double scalingTangentialForce = 0.4;
        const double forceX = forceZ * scalingTangentialForce;
        const double forceY = -forceZ * scalingTangentialForce;
        const double torqueX = forceZ * CoP(1);
        const double torqueY = -forceZ * CoP(0);
        const double torqueZ = 0;

        BipedalLocomotion::Math::Wrenchd wrench;
        wrench << forceX, forceY, forceZ, torqueX, torqueY, torqueZ;

        // Check unfeasibility
        // Ax - b > 0
        Eigen::VectorXd constraintValue = -cone.getB();
        constraintValue.noalias() += cone.getA() * wrench;
        REQUIRE((constraintValue.array() > 0.0).any());
    }
}
