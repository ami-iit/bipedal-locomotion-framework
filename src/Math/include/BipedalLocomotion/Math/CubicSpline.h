/**
 * @file CubicSpline.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_MATH_CUBIC_SPLINE_H
#define BIPEDAL_LOCOMOTION_MATH_CUBIC_SPLINE_H

#include <chrono>
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <BipedalLocomotion/Math/Spline.h>

namespace BipedalLocomotion
{
namespace Math
{
/**
 * Cubic spline implement a 3-rd order polynomial spline in \$f\mathbb{R}^n\$f.
 * @note The spline is defined as a set of piecewise polynomial functions of degree 3 of the form
 * \f[
 * s(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3
 * \f]
 * where \$t \in [0, T_i]\$ and \$T_i\$ is the duration of the i-th polynomial.
 */
template <typename T> class CubicSpline : public Spline<T>
{
    /**
     * Update the polynomial coefficients.
     * @param polynomial polynomial to be updated.
     * @note this function is called by the Spline::computeCoefficients() function.
     */
    void updatePolynomialCoefficients(typename Spline<T>::Polynomial& polynomial) final;

    /**
     * Compute the intermediate quantities.
     * @note this function is called by the Spline::computeCoefficients() function.
     */
    void computeIntermediateQuantities() final;

    /**
     * Convert a duration to seconds.
     * @param d duration to be converted.
     * @return the duration in seconds.
     */
    template <class Rep, class Period>
    constexpr double durationToSeconds(const std::chrono::duration<Rep, Period>& d)
    {
        return std::chrono::duration<double>(d).count();
    };

    /**
     * addTripletCurrentKnot is a helper function to generate a triplet containing a 2x2 matrix.
     * This matrix is used to compute the intermediate velocity and acceleration.
     */
    void addTripletCurrentKnot(const int knotIndex,
                               const int rowOffset,
                               const int columnOffset,
                               std::vector<Eigen::Triplet<typename T::Scalar>>& tripletList);

    /**
     * addTripletPreviousKnot is a helper function to generate a triplet containing a 2x2 matrix.
     * This matrix is used to compute the intermediate velocity and acceleration.
     */
    void addTripletPreviousKnot(const int knotIndex,
                                const int rowOffset,
                                const int columnOffset,
                                std::vector<Eigen::Triplet<typename T::Scalar>>& tripletList);
    /**
     * addTripletNextKnot is a helper function to generate a triplet containing a 2x2 matrix.
     * This matrix is used to compute the intermediate velocity and acceleration.
     */
    void addTripletNextKnot(const int knotIndex,
                            const int rowOffset,
                            const int columnOffset,
                            std::vector<Eigen::Triplet<typename T::Scalar>>& tripletList);

    /**
     * addKnownTermKnotPosition is a helper function to generate a 2-d vector,
     * This vector is the known term used to compute the intermediate velocity and acceleration.
     */
    void addKnownTermKnotPosition(const std::size_t knotIndex,
                                  const std::size_t coordinateIndex,
                                  Eigen::Ref<Eigen::Matrix<typename T::Scalar, 1, 1>> b);

    /**
     * addKnownTermNextKnot is a helper function to generate a 2-d vector,
     * This vector is the known term used to compute the intermediate velocity and acceleration.
     */
    void addKnownTermNextKnot(const std::size_t knotIndex,
                              const std::size_t coordinateIndex,
                              Eigen::Ref<Eigen::Matrix<typename T::Scalar, 1, 1>> b);

    /**
     * addKnownTermPreviousKnot is a helper function to generate a 2-d vector,
     * This vector is the known term used to compute the intermediate velocity and acceleration.
     */
    void addKnownTermPreviousKnot(const std::size_t knotIndex,
                                  const std::size_t coordinateIndex,
                                  Eigen::Ref<Eigen::Matrix<typename T::Scalar, 1, 1>> b);
};

template <typename T>
void CubicSpline<T>::updatePolynomialCoefficients(typename Spline<T>::Polynomial& poly)
{
    const double d = std::chrono::duration<double>(poly.duration).count();

    const auto& x0 = poly.initialPoint->position;
    const auto& dx0 = poly.initialPoint->velocity;

    const auto& xT = poly.finalPoint->position;
    const auto& dxT = poly.finalPoint->velocity;

    // resize the coefficients
    poly.coefficients.resize(4);
    poly.coefficients[0] = x0;
    poly.coefficients[1] = dx0;
    poly.coefficients[2] = -(3 * x0 - 3 * xT + 2 * d * dx0 + d * dxT) / (d * d);
    poly.coefficients[3] = (2 * x0 - 2 * xT + d * dx0 + d * dxT) / (d * d * d);
}

template <typename T> void CubicSpline<T>::computeIntermediateQuantities()
{
    // here we assume that at least 3 points has been defined
    const std::size_t numberOfInteriorKnots = this->m_knots.size() - 2;

    Eigen::SparseMatrix<typename T::Scalar> A(numberOfInteriorKnots, numberOfInteriorKnots);
    std::vector<Eigen::Triplet<typename T::Scalar>> tripletsList;

    // Given a set of interior points the we can define a matrix A as
    //                          __                                        __
    //                         | x x 0 0 0 0 0 0 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //                         | x x x 0 0 0 0 0 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //                         | 0 x x x 0 0 0 0 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //     /\      ______      | 0 0 x x x 0 0 0 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //    /  \    |______|     | 0 0 0 x x x 0 0 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //   / /\ \    ______      |         ........................          |
    //  / ____ \  |______|     |         ........................          |
    // /_/    \_\              |         ........................          |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... x x x 0 0 0 0 |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... 0 x x x 0 0 0 |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... 0 0 x x x 0 0 |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... 0 0 0 x x x 0 |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... 0 0 0 0 x x x |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... 0 0 0 0 0 x x |
    //                         |__                                       __|
    //
    // where x represents a non zero number and 0 represents the number 0. The matrix A can be
    // stored as a sparse matrix where the number of non zero entries for the extremum elements are
    // 2 and the number of non zero elements for the interior (non extremum elements) are 3
    // The matrix A is a square matrix whose number of columns (and rows) is equal to the
    // number of interior knots of the spline times. If there is only an interior knot, A is a 1x1
    // dense matrix.
    unsigned int numberOfExpectedTriplets = 1;
    if (numberOfInteriorKnots > 1)
    {
        constexpr std::size_t numberOfExtremumElements = 2;
        constexpr std::size_t numberOfNonExtremumElements = 4;
        constexpr std::size_t numberOfExtremum = 2;
        numberOfExpectedTriplets = numberOfNonExtremumElements //
                                       * (numberOfInteriorKnots - numberOfExtremum)
                                   + numberOfExtremum * numberOfExtremumElements;
    }
    tripletsList.reserve(numberOfExpectedTriplets);

    for (int i = 0; i < numberOfInteriorKnots; i++)
    {
        const int absoluteKnotIndex = i + 1;

        // in this case there is only one interior knot A will be a 2
        if (i == 0 && (i + 1) == numberOfInteriorKnots)
        {
            this->addTripletCurrentKnot(absoluteKnotIndex, 0, 0, tripletsList);
        } else if (i == 0)
        {
            this->addTripletCurrentKnot(absoluteKnotIndex, i, 0, tripletsList);
            this->addTripletNextKnot(absoluteKnotIndex, i, 1, tripletsList);
        } else if (i + 1 == numberOfInteriorKnots)
        {
            this->addTripletPreviousKnot(absoluteKnotIndex, i, (i - 1), tripletsList);
            this->addTripletCurrentKnot(absoluteKnotIndex, i, (i - 1) + 1, tripletsList);
        } else
        {
            this->addTripletPreviousKnot(absoluteKnotIndex, i, (i - 1), tripletsList);
            this->addTripletCurrentKnot(absoluteKnotIndex, i, (i - 1) + 1, tripletsList);
            this->addTripletNextKnot(absoluteKnotIndex, i, (i - 1) + 1 + 1, tripletsList);
        }
    }

    // create the sparse matrix
    A.setFromTriplets(tripletsList.begin(), tripletsList.end());

    // compute first coordinate
    Eigen::Matrix<typename T::Scalar, Eigen::Dynamic, 1> b(A.rows());
    for (size_t j = 0; j < this->m_knots.front().position.size(); j++)
    {
        b.setZero();
        for (size_t i = 0; i < numberOfInteriorKnots; i++)
        {
            const int absoluteKnotIndex = i + 1;

            if (i == 0 && (i + 1) == numberOfInteriorKnots)
            {
                this->addKnownTermKnotPosition(absoluteKnotIndex, j, b.template head<1>());
                this->addKnownTermNextKnot(absoluteKnotIndex, j, b.template head<1>());
                this->addKnownTermPreviousKnot(absoluteKnotIndex, j, b.template head<1>());

            } else if (i == 0)
            {
                this->addKnownTermKnotPosition(absoluteKnotIndex, j, b.template head<1>());
                this->addKnownTermPreviousKnot(absoluteKnotIndex, j, b.template head<1>());

            } else if (i + 1 == numberOfInteriorKnots)
            {
                this->addKnownTermNextKnot(absoluteKnotIndex, j, b.template segment<1>(i));
                this->addKnownTermKnotPosition(absoluteKnotIndex, j, b.template segment<1>(i));
            } else
            {
                this->addKnownTermKnotPosition(absoluteKnotIndex, j, b.template segment<1>(i));
            }
        }

        using EigenSparse = Eigen::SparseMatrix<typename T::Scalar>;
        using Ordering = typename Eigen::COLAMDOrdering<typename EigenSparse::StorageIndex>;

        Eigen::SparseQR<EigenSparse, Ordering> qrDecomposition;
        qrDecomposition.compute(A);
        Eigen::VectorXd solution = qrDecomposition.solve(b);

        for (size_t i = 0; i < numberOfInteriorKnots; i++)
        {
            const int absoluteKnotIndex = i + 1;

            this->m_knots[absoluteKnotIndex].velocity[j] = solution[i];
        }
    }
}

template <typename T>
void CubicSpline<T>::addTripletCurrentKnot(
    const int knotIndex,
    const int rowOffset,
    const int columnOffset,
    std::vector<Eigen::Triplet<typename T::Scalar>>& tripletList)
{
    const auto& poly = this->m_polynomials[knotIndex];
    const auto& prevPoly = this->m_polynomials[knotIndex - 1];

    // The following triplet represent this matrix
    //  /      /   1      1\ \
    //  |   4  |------ + --| |
    //  |      |T        T | |
    //  |      \ i - 1    i/ |
    //  \                    /

    tripletList.emplace_back(rowOffset,
                             columnOffset,
                             4
                                 * (1 / durationToSeconds(prevPoly.duration)
                                    + 1 / durationToSeconds(poly.duration)));
}

template <typename T>
void CubicSpline<T>::addTripletPreviousKnot(
    const int knotIndex,
    const int rowOffset,
    const int columnOffset,
    std::vector<Eigen::Triplet<typename T::Scalar>>& tripletList)
{
    const auto& poly = this->m_polynomials[knotIndex - 1];

    // The following triplet represent this matrix
    // /     2     \
    // |  -------   |
    // |  T         |
    // \   i - 1    /

    tripletList.emplace_back(rowOffset, columnOffset, 2 / durationToSeconds(poly.duration));
}

template <typename T>
void CubicSpline<T>::addTripletNextKnot(const int knotIndex,
                                        const int rowOffset,
                                        const int columnOffset,
                                        std::vector<Eigen::Triplet<typename T::Scalar>>& tripletList)
{
    const auto& poly = this->m_polynomials[knotIndex];

    // The following triplet represent this matrix
    // /    2    \
    // |   ---    |
    // |   T      |
    // \    i     /

    tripletList.emplace_back(rowOffset, columnOffset, 2 / durationToSeconds(poly.duration));
}

template <typename T>
void CubicSpline<T>::addKnownTermKnotPosition(const std::size_t knotIndex,
                                              const std::size_t coordinateIndex,
                                              Eigen::Ref<Eigen::Matrix<typename T::Scalar, 1, 1>> b)
{
    const auto& poly = this->m_polynomials;
    const auto& knots = this->m_knots;

    const auto& i = knotIndex;
    const auto& j = coordinateIndex;

    b[0] += 6
            * ((knots[i].position[j] - knots[i - 1].position[j])
                   / std::pow(durationToSeconds(poly[i - 1].duration), 2)
               + (knots[i + 1].position[j] - knots[i].position[j])
                     / std::pow(durationToSeconds(poly[i].duration), 2));
}

template <typename T>
void CubicSpline<T>::addKnownTermNextKnot(const std::size_t knotIndex,
                                          const std::size_t coordinateIndex,
                                          Eigen::Ref<Eigen::Matrix<typename T::Scalar, 1, 1>> b)
{
    const auto& poly = this->m_polynomials;
    const auto& knots = this->m_knots;

    const std::size_t i = knotIndex;
    const std::size_t j = coordinateIndex;
    b[0] -= 2 * knots[i + 1].velocity[j] / durationToSeconds(poly[i].duration);
}

template <typename T>
void CubicSpline<T>::addKnownTermPreviousKnot(std::size_t knotIndex,
                                              std::size_t coordinateIndex,
                                              Eigen::Ref<Eigen::Matrix<typename T::Scalar, 1, 1>> b)
{
    const auto& poly = this->m_polynomials;
    const auto& knots = this->m_knots;

    const std::size_t i = knotIndex;
    const std::size_t j = coordinateIndex;

    b[0] -= 2 * knots[i - 1].velocity[j] / durationToSeconds(poly[i - 1].duration);
}

} // namespace Math
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATH_CUBIC_SPLINE_H
