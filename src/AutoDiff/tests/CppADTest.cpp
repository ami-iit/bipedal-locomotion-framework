/**
 * @file CppADTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/AutoDiff/CppAD.h>

TEST_CASE("CppAD and Eigen")
{
    // Some coefficients useful for the test
    constexpr double a = 4;
    constexpr double b = 1.1341;
    constexpr double c = 2.3213;

    BipedalLocomotion::AutoDiff::CppAD::VectorXAD x(3);

    // Start recording
    CppAD::Independent(x);
    BipedalLocomotion::AutoDiff::CppAD::VectorXAD y = a * x.array() + c * (b * x.array()).sin();

    // stop recording
    CppAD::ADFun<double> f(x, y);

    Eigen::VectorXd xNum = Eigen::VectorXd::Random(3);
    Eigen::VectorXd jac = f.Jacobian(xNum);

    // The Jacobian is stored as a row-major vector
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacMatrix(jac.data());

    // Compute the analytic Jacobian
    auto analyticJacobian = [&a, &b, &c](const Eigen::Ref<Eigen::Vector3d>& x) -> Eigen::Matrix3d {
        Eigen::Vector3d jacobian;
        jacobian = a + c * b * (b * x.array()).cos();
        return jacobian.asDiagonal();
    };

    constexpr double tolerance = 1e-4;
    REQUIRE(jacMatrix.isApprox(analyticJacobian(xNum), tolerance));
}
