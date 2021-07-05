/**
 * @file FrictionConesTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotion/Math/LinearizedFrictionCone.h>
#include <BipedalLocomotion/Math/ContactWrenchCone.h>
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
