/**
 * @file AlgebraicRiccatiEquationTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotion/Math/AlgebraicRiccatiEquation.h>

using namespace BipedalLocomotion::Math;
using Matrix1d = Eigen::Matrix<double, 1, 1>;

TEST_CASE("Algebraic Riccati Equation")
{
    Eigen::Matrix3d A;
    A << 0, 1, 0,
         0, 0, 0,
         0, 0, 1;

    Eigen::Vector3d b;
    b << 0,
         1,
         1;

    const Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
    const Matrix1d R = Matrix1d::Identity();

    const std::pair<bool, Eigen::MatrixXd> solution = ContinuousAlgebraicRiccatiEquation(A, b, Q, R);

    REQUIRE(solution.first);
    Eigen::Matrix3d matlabSolution;
    matlabSolution << 3.8051,    6.7394,   -7.7394,
                      6.7394,   17.9048,  -21.7099,
                     -7.7394,  -21.7099,   29.4494;

    constexpr double tolerance = 1e-5;
    REQUIRE(matlabSolution.isApprox(solution.second, tolerance));
}
