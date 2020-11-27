/**
 * @file AlgebraicRiccatiEquation.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iostream>
#include <sstream>

#include <BipedalLocomotion/Math/AlgebraicRiccatiEquation.h>


std::pair<bool, Eigen::MatrixXd>
BipedalLocomotion::Math::ContinuousAlgebraicRiccatiEquation(Eigen::Ref<const Eigen::MatrixXd> A,
                                                            Eigen::Ref<const Eigen::MatrixXd> B,
                                                            Eigen::Ref<const Eigen::MatrixXd> Q,
                                                            Eigen::Ref<const Eigen::MatrixXd> R,
                                                            double tolerance,
                                                            std::size_t maxIterations,
                                                            bool verbose)
{
    constexpr std::string_view errorPrefix = "[BipedalLocomotion::Math::"
                                             "ContinuousAlgebraicRiccatiEquation] ";

    std::pair<bool, Eigen::MatrixXd> result;

    const std::size_t n = B.rows();
    const std::size_t m = B.cols();

    // check the size of the matrices
    if (A.rows() != n || A.cols() != n)
    {
        std::cerr << errorPrefix << "The matrix A must be a square matrix. Expected size: " << n
                  << " x " << n << ". Passed size: " << A.rows() << " x " << A.cols() << "."
                  << std::endl;
        result.first = false;
        return result;
    }

    if (Q.rows() != n || Q.cols() != n)
    {
        std::cerr << errorPrefix << "The matrix Q must be a square matrix. Expected size: " << n
                  << " x " << n << ". Passed size: " << Q.rows() << " x " << Q.cols() << "."
                  << std::endl;
        result.first = false;
        return result;
    }

    if (R.rows() != m || R.cols() != m)
    {
        std::cerr << errorPrefix << "The matrix Q must be a square matrix. Expected size: " << n
                  << " x " << n << ". Passed size: " << Q.rows() << " x " << Q.cols() << "."
                  << std::endl;
        result.first = false;
        return result;
    }

    Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
    if (R_cholesky.info() != Eigen::Success)
    {
        std::cerr << errorPrefix << "The matrix R must be positive definite." << std::endl;
        result.first = false;
        return result;
    }

    constexpr double symmetricTolerance = 1e-8;
    if (!Q.isApprox(Q.transpose(), symmetricTolerance))
    {
        std::cerr << errorPrefix << "The matrix Q must be symmetric." << std::endl;
        result.first = false;
        return result;
    }

    Eigen::MatrixXd H(2 * n, 2 * n);
    H << A, B * R_cholesky.solve(B.transpose()),
         Q, -A.transpose();

    Eigen::MatrixXd Z = H;
    Eigen::MatrixXd Z_old;

    double relativeNorm = tolerance;
    std::size_t iteration = 0;
    const double p = static_cast<double>(Z.rows());

    for (; iteration < maxIterations && relativeNorm >= tolerance;
         iteration++)
    {
        Z_old = Z;
        // R. Byers. Solving the algebraic Riccati equation with the matrix sign
        // function. Linear Algebra Appl., 85:267–279, 1987
        // Added determinant scaling to improve convergence (converges in rough half
        // the iterations with this)
        const double ck = std::pow(std::abs(Z.determinant()), -1.0 / p);
        Z *= ck;
        Z = Z - 0.5 * (Z - Z.inverse());
        relativeNorm = (Z - Z_old).norm();
    }

    if (verbose)
    {
        std::stringstream info;
        if (relativeNorm < tolerance)
            info << "Solution found.";
        else
            info << "Solution not found.";
        info << std::endl;
        info << "Number of iteration: " << iteration << ". Relative error: " << relativeNorm << ".";

        std::cout << errorPrefix << info.str() << std::endl;
    }

    Eigen::Ref<const Eigen::MatrixXd> W11 = Z.block(0, 0, n, n);
    Eigen::Ref<const Eigen::MatrixXd> W12 = Z.block(0, n, n, n);
    Eigen::Ref<const Eigen::MatrixXd> W21 = Z.block(n, 0, n, n);
    Eigen::Ref<const Eigen::MatrixXd> W22 = Z.block(n, n, n, n);

    Eigen::MatrixXd lhs(2 * n, n);
    Eigen::MatrixXd rhs(2 * n, n);
    const Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(n, n);
    lhs << W12, W22 + eye;
    rhs << W11 + eye, W21;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(lhs, Eigen::ComputeThinU | Eigen::ComputeThinV);
    result.first = true;
    result.second = svd.solve(rhs);

    return result;
}
