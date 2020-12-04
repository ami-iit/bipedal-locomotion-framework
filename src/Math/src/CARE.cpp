/**
 * @file CARE.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iostream>
#include <sstream>

#include <BipedalLocomotion/Math/CARE.h>

using namespace BipedalLocomotion::Math;

struct CARE::Impl
{
    double tolerance{1e-9};
    bool isVerbose{false};
    int maxIterations{100};

    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

    Eigen::MatrixXd lhs;
    Eigen::MatrixXd rhs;
    Eigen::MatrixXd Z;
    Eigen::MatrixXd ZOld;
    Eigen::MatrixXd ZInverse;

    Eigen::MatrixXd identity;
    Eigen::MatrixXd solution;

    void resizeInteralMatrices()
    {
        const std::size_t n = B.rows();

        // resize the Hamiltonian matrix
        Z.resize(2 * n, 2 * n);
        ZOld.resize(2 * n, 2 * n);
        ZInverse.resize(2 * n, 2 * n);

        identity.resize(n, n);
        identity.setIdentity();
        lhs.resize(2 * n, n);
        rhs.resize(2 * n, n);
    }
};

CARE::CARE()
{
    m_pimpl = std::make_unique<Impl>();
}

CARE::~CARE() = default;

bool CARE::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    // all the parameters are optional
    auto ptr = handler.lock();

    if(ptr == nullptr)
        return true;

    ptr->getParameter("tolerance", m_pimpl->tolerance);
    ptr->getParameter("is_verbose", m_pimpl->isVerbose);
    ptr->getParameter("max_iterations", m_pimpl->maxIterations);

    return true;
}

bool CARE::setMatrices(Eigen::Ref<const Eigen::MatrixXd> A,
                       Eigen::Ref<const Eigen::MatrixXd> B,
                       Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::MatrixXd> R)
{
    constexpr std::string_view errorPrefix = "[CARE::setMatrices] ";

    const std::size_t n = B.rows();
    const std::size_t m = B.cols();

    // check the size of the matrices
    if (A.rows() != n || A.cols() != n)
    {
        std::cerr << errorPrefix << "The matrix A must be a square matrix. Expected size: " << n
                  << " x " << n << ". Passed size: " << A.rows() << " x " << A.cols() << "."
                  << std::endl;
        return false;
    }

    if (Q.rows() != n || Q.cols() != n)
    {
        std::cerr << errorPrefix << "The matrix Q must be a square matrix. Expected size: " << n
                  << " x " << n << ". Passed size: " << Q.rows() << " x " << Q.cols() << "."
                  << std::endl;
        return false;
    }

    if (R.rows() != m || R.cols() != m)
    {
        std::cerr << errorPrefix << "The matrix R must be a square matrix. Expected size: " << m
                  << " x " << m << ". Passed size: " << R.rows() << " x " << R.cols() << "."
                  << std::endl;
        return false;
    }

    // this check may be time consuming let's consider it only when the library is compiled in debug
    constexpr double symmetricTolerance = 1e-8;
    assert(Q.isApprox(Q.transpose(), symmetricTolerance) && "The matrix Q must be symmetric");

    m_pimpl->A = A;
    m_pimpl->B = B;
    m_pimpl->Q = Q;
    m_pimpl->R = R;

    m_pimpl->resizeInteralMatrices();

    return true;
}

bool CARE::solve()
{
    const std::size_t n = m_pimpl->A.rows();

    // compute cholesky decomposition
    Eigen::LLT<Eigen::MatrixXd> R_cholesky(m_pimpl->R);
    if (R_cholesky.info() != Eigen::Success)
    {
        std::cerr << "[CARE::solve] The matrix R must be positive definite." << std::endl;
        return false;
    }

    // Z represents the Hamiltonian matrix.
    //       _                _
    //      | A   B inv(R) B'  |
    // Z  = |                  |
    //      |_Q       -  A'   _|

    m_pimpl->Z << m_pimpl->A, m_pimpl->B * R_cholesky.solve(m_pimpl->B.transpose()),
                  m_pimpl->Q, -m_pimpl->A.transpose();

    double relativeNorm = m_pimpl->tolerance;
    std::size_t iteration = 0;
    const double p = static_cast<double>(m_pimpl->Z.rows());

    // run the algorithm
    for (; iteration < m_pimpl->maxIterations && relativeNorm >= m_pimpl->tolerance; iteration++)
    {
        m_pimpl->ZOld = m_pimpl->Z;

        // R. Byers. Solving the algebraic Riccati equation with the matrix sign
        // function. Linear Algebra Appl., 85:267–279, 1987
        // Added determinant scaling to improve convergence (converges in rough half
        // the iterations with this)
        const double ck = std::pow(std::abs(m_pimpl->Z.determinant()), -1.0 / p);
        m_pimpl->Z *= ck;
        m_pimpl->ZInverse = m_pimpl->Z.inverse();
        m_pimpl->Z = m_pimpl->Z - 0.5 * (m_pimpl->Z - m_pimpl->ZInverse);
        relativeNorm = (m_pimpl->Z - m_pimpl->ZOld).norm();
    }

    const std::size_t n = m_pimpl->A.rows();

    Eigen::Ref<const Eigen::MatrixXd> W11 = m_pimpl->Z.block(0, 0, n, n);
    Eigen::Ref<const Eigen::MatrixXd> W12 = m_pimpl->Z.block(0, n, n, n);
    Eigen::Ref<const Eigen::MatrixXd> W21 = m_pimpl->Z.block(n, 0, n, n);
    Eigen::Ref<const Eigen::MatrixXd> W22 = m_pimpl->Z.block(n, n, n, n);

    m_pimpl->lhs << W12,
                    W22 + m_pimpl->identity;
    m_pimpl->rhs << W11 + m_pimpl->identity,
                    W21;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(m_pimpl->lhs, Eigen::ComputeThinU | Eigen::ComputeThinV);
    m_pimpl->solution = svd.solve(m_pimpl->rhs);

    if (m_pimpl->isVerbose)
    {
        std::stringstream info;
        if (relativeNorm < m_pimpl->tolerance)
            info << "Solution found.";
        else
            info << "Solution not found.";
        info << std::endl;
        info << "Number of iteration: " << iteration << ". Relative error: " << relativeNorm << ".";

        std::cout << "[CARE::solve] " << info.str() << std::endl;
    }

    return relativeNorm < m_pimpl->tolerance;
}

Eigen::Ref<const Eigen::MatrixXd> CARE::getSolution() const
{
    return m_pimpl->solution;
}
