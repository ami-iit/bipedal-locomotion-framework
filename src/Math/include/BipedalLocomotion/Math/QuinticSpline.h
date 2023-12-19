/**
 * @file QuinticSpline.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_MATH_QUINTIC_SPLINE_H
#define BIPEDAL_LOCOMOTION_MATH_QUINTIC_SPLINE_H

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
 * Quintic spline implements a 5-th order polynomial spline in \f$\mathbb{R}^n\f$.
 * @note The spline is defined as a set piecewise polynomial functions of degree 5 of the form
 * \f[
 * s_i(t) = a_i + b_i t + c_i t^2 + d_i t^3 + e_i t^4 + f_i t^5
 * \f]
 * where \f$t \in [0, T_i]\f$ and \f$T_i\f$ is the duration of the i-th polynomial.
 */
template <typename T> class QuinticSpline : public Spline<T>
{
    /**
     * Compute the coefficients of the polynomial.
     * @param poly polynomial to be updated.
     * @note this function is called by the Spline::computeCoefficients() function.
     */
    void updatePolynomialCoefficients(typename Spline<T>::Polynomial& polynomial) final;

    /**
     * Compute the intermediate quantities of the spline.
     * @note this function is called by the Spline::computeCoefficients() function.
     */
    void computeIntermediateQuantities() final;

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

    template <class Rep, class Period>
    constexpr double durationToSeconds(const std::chrono::duration<Rep, Period>& d)
    {
        return std::chrono::duration<double>(d).count();
    };

    void addKnownTermKnotPosition(std::size_t knotIndex, //
                                  std::size_t coordinateIndex,
                                  Eigen::Ref<Eigen::Matrix<typename T::Scalar, 2, 1>> b);

    void addKnownTermNextKnot(std::size_t knotIndex, //
                              std::size_t coordinateIndex,
                              Eigen::Ref<Eigen::Matrix<typename T::Scalar, 2, 1>> b);

    void addKnownTermPreviousKnot(std::size_t knotIndex, //
                                  std::size_t coordinateIndex,
                                  Eigen::Ref<Eigen::Matrix<typename T::Scalar, 2, 1>> b);
};

template <typename T>
void QuinticSpline<T>::addTripletCurrentKnot(
    const int knotIndex,
    const int rowOffset,
    const int columnOffset,
    std::vector<Eigen::Triplet<typename T::Scalar>>& tripletList)
{
    const auto& poly = this->m_polynomials[knotIndex];
    const auto& prevPoly = this->m_polynomials[knotIndex - 1];

    const double prevPolyDuration = durationToSeconds(prevPoly.duration);
    const double polyDuration = durationToSeconds(poly.duration);

    // The following triplets represent this matrix
    // /    /   1      1\          /   1      1\ \
    // | 12 |------ - --|      -3  |------ + --| |
    // |    | 2        2|          |T        T | |
    // |    |T        T |          \ i - 1    i/ |
    // |    \ i - 1    i/                        |
    // |                                         |
    // |                                         |
    // |    /   1      1\         / 1      1  \  |
    // | 16 |------ + --|       3 |-- - ------|  |
    // |    | 3        3|         | 2    2    |  |
    // |    |T        T |         |T    T     |  |
    // \    \ i - 1    i/         \ i    i - 1/  /

    tripletList.push_back(
        {rowOffset,
         columnOffset,
         12 * ((1 / std::pow(prevPolyDuration, 2)) - (1 / std::pow(polyDuration, 2)))});

    tripletList.push_back(
        {rowOffset, columnOffset + 1, -3 * ((1 / prevPolyDuration) + (1 / polyDuration))});

    tripletList.push_back(
        {rowOffset + 1,
         columnOffset,
         16 * ((1 / std::pow(prevPolyDuration, 3)) + (1 / std::pow(polyDuration, 3)))});

    tripletList.push_back(
        {rowOffset + 1,
         columnOffset + 1,
         3 * ((-1 / std::pow(prevPolyDuration, 2)) + (1 / std::pow(polyDuration, 2)))});
}

template <typename T>
void QuinticSpline<T>::addTripletPreviousKnot(
    const int knotIndex,
    const int rowOffset,
    const int columnOffset,
    std::vector<Eigen::Triplet<typename T::Scalar>>& tripletList)
{
    const auto& poly = this->m_polynomials[knotIndex - 1];
    const double polyDuration = durationToSeconds(poly.duration);

    // The following triplets represent this matrix
    // /    8          1    \
    // | ------     ------  |
    // |  2                 |
    // | T           T      |
    // |  i - 1       i - 1 |
    // |                    |
    // |                    |
    // |   14          2    |
    // | ------     ------  |
    // |  3          2      |
    // | T          T       |
    // \  i - 1      i - 1  /

    tripletList.push_back({rowOffset, columnOffset, 8 / std::pow(polyDuration, 2)});
    tripletList.push_back({rowOffset, columnOffset + 1, 1 / polyDuration});
    tripletList.push_back({rowOffset + 1, columnOffset, 14 / std::pow(polyDuration, 3)});
    tripletList.push_back({rowOffset + 1, columnOffset + 1, 2 / std::pow(polyDuration, 2)});
}

template <typename T>
void QuinticSpline<T>::addTripletNextKnot(
    const int knotIndex,
    const int rowOffset,
    const int columnOffset,
    std::vector<Eigen::Triplet<typename T::Scalar>>& tripletList)
{
    const auto& poly = this->m_polynomials[knotIndex];
    const double polyDuration = durationToSeconds(poly.duration);

    // The following triplets represent this matrix
    // /    8          1    \
    // | - ---        ---   |
    // |    2               |
    // |   T           T    |
    // |    i           i   |
    // |                    |
    // |                    |
    // |  14           2    |
    // |  --         - ---  |
    // |   3            2   |
    // |  T            T    |
    // \   i            i   /

    tripletList.push_back({rowOffset, columnOffset, -8 / std::pow(polyDuration, 2)});
    tripletList.push_back({rowOffset, columnOffset + 1, 1 / polyDuration});
    tripletList.push_back({rowOffset + 1, columnOffset, 14 / std::pow(polyDuration, 3)});
    tripletList.push_back({rowOffset + 1, columnOffset + 1, -2 / std::pow(polyDuration, 2)});
}

template <typename T>
void QuinticSpline<T>::addKnownTermKnotPosition(
    std::size_t knotIndex,
    std::size_t coordinateIndex,
    Eigen::Ref<Eigen::Matrix<typename T::Scalar, 2, 1>> b)
{
    const auto& poly = this->m_polynomials;

    const std::size_t i = knotIndex;
    const std::size_t j = coordinateIndex;

    b[0] += 20
            * (1 / std::pow(durationToSeconds(poly[i - 1].duration), 3)
                   * (this->m_knots[i].position[j] - this->m_knots[i - 1].position[j])
               + 1 / std::pow(durationToSeconds(poly[i].duration), 3)
                     * (this->m_knots[i].position[j] - this->m_knots[i + 1].position[j]));

    b[1] += 30
            * (1 / std::pow(durationToSeconds(poly[i - 1].duration), 4)
                   * (this->m_knots[i].position[j] - this->m_knots[i - 1].position[j])
               - 1 / std::pow(durationToSeconds(poly[i].duration), 4)
                     * (this->m_knots[i].position[j] - this->m_knots[i + 1].position[j]));
}

template <typename T>
void QuinticSpline<T>::addKnownTermNextKnot(std::size_t knotIndex,
                                            std::size_t coordinateIndex,
                                            Eigen::Ref<Eigen::Matrix<typename T::Scalar, 2, 1>> b)
{
    const auto& poly = this->m_polynomials;

    const auto& i = knotIndex;
    const auto& j = coordinateIndex;

    Eigen::Matrix<typename T::Scalar, 2, 2> tempMatrix;
    tempMatrix << -8 / std::pow(durationToSeconds(poly[i].duration), 2),
        1 / durationToSeconds(poly[i].duration),
        14 / std::pow(durationToSeconds(poly[i].duration), 3),
        -2 / std::pow(durationToSeconds(poly[i].duration), 2);

    Eigen::Matrix<typename T::Scalar, 2, 1> tempVector;
    tempVector << this->m_knots[i + 1].velocity[j], this->m_knots[i + 1].acceleration[j];
    b -= tempMatrix * tempVector;
}

template <typename T>
void QuinticSpline<T>::addKnownTermPreviousKnot(
    std::size_t knotIndex,
    std::size_t coordinateIndex,
    Eigen::Ref<Eigen::Matrix<typename T::Scalar, 2, 1>> b)
{
    const auto& poly = this->m_polynomials;

    const auto& i = knotIndex;
    const auto& j = coordinateIndex;

    Eigen::Matrix<typename T::Scalar, 2, 2> tempMatrix;
    tempMatrix << 8 / std::pow(durationToSeconds(poly[i - 1].duration), 2),
        1 / durationToSeconds(poly[i - 1].duration),
        14 / std::pow(durationToSeconds(poly[i - 1].duration), 3),
        2 / std::pow(durationToSeconds(poly[i - 1].duration), 2);

    Eigen::Matrix<typename T::Scalar, 2, 1> tempVector;
    tempVector << this->m_knots[i - 1].velocity[j], this->m_knots[i - 1].acceleration[j];
    b -= tempMatrix * tempVector;
}

template <typename T>
void QuinticSpline<T>::updatePolynomialCoefficients(typename Spline<T>::Polynomial& poly)
{
    const double d = std::chrono::duration<double>(poly.duration).count();

    const auto& x0 = poly.initialPoint->position;
    const auto& dx0 = poly.initialPoint->velocity;
    const auto& ddx0 = poly.initialPoint->acceleration;

    const auto& xT = poly.finalPoint->position;
    const auto& dxT = poly.finalPoint->velocity;
    const auto& ddxT = poly.finalPoint->acceleration;

    // resize the coefficients
    poly.coefficients.resize(6);
    poly.coefficients[0] = x0;
    poly.coefficients[1] = dx0;
    poly.coefficients[2] = ddx0 / 2.0;
    poly.coefficients[3] = 1.0 / (d * d * d)
                           * (x0 * 2.0E+1 - xT * 2.0E+1 + d * dx0 * 1.2E+1 + d * dxT * 8.0
                              + (d * d) * ddx0 * 3.0 - (d * d) * ddxT)
                           * (-1.0 / 2.0);
    poly.coefficients[4] = (1.0 / (d * d * d * d)
                            * (x0 * 3.0E+1 - xT * 3.0E+1 + d * dx0 * 1.6E+1 + d * dxT * 1.4E+1
                               + (d * d) * ddx0 * 3.0 - (d * d) * ddxT * 2.0))
                           / 2.0;
    poly.coefficients[5] = 1.0 / (d * d * d * d * d)
                           * (x0 * 1.2E+1 - xT * 1.2E+1 + d * dx0 * 6.0 + d * dxT * 6.0
                              + (d * d) * ddx0 - (d * d) * ddxT)
                           * (-1.0 / 2.0);
}

template <typename T> void QuinticSpline<T>::computeIntermediateQuantities()
{
    // here we assume that at least 3 points has been defined
    const std::size_t numberOfInteriorKnots = this->m_knots.size() - 2;

    Eigen::SparseMatrix<typename T::Scalar> A(2 * numberOfInteriorKnots, 2 * numberOfInteriorKnots);
    std::vector<Eigen::Triplet<typename T::Scalar>> tripletsList;

    // Given a set of interior points the we can define a matrix A as
    //                          __                                        __
    //                         | x x x x 0 0 0 0 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //                         | x x x x 0 0 0 0 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //                         | x x x x x x 0 0 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //                         | x x x x x x 0 0 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //                         | 0 0 x x x x x x 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //                         | 0 0 x x x x x x 0 0 0 0 ... 0 0 0 0 0 0 0 |
    //     /\      ______      | 0 0 0 0 x x x x x x 0 0 ... 0 0 0 0 0 0 0 |
    //    /  \    |______|     | 0 0 0 0 x x x x x x 0 0 ... 0 0 0 0 0 0 0 |
    //   / /\ \    ______      |         ........................          |
    //  / ____ \  |______|     |         ........................          |
    // /_/    \_\              |         ........................          |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... x x x x x x x |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... x x x x x x x |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... 0 0 x x x x x |
    //                         | 0 0 0 0 0 0 0 0 0 0 0 0 ... 0 0 x x x x x |
    //                         |__                                       __|
    //
    // where x represents a non zero number and 0 represents the number 0. The matrix A can be
    // stored as a sparse matrix where the number of non zero entries for the extremum elements are
    // 4 * 2 and the number of non zero elements for the interior (non extremum elements) are 6
    // * 2. The matrix A is a square matrix whose number of columns (and rows) is equal to the
    // number of interior knots of the spline times 2. If there is only an interior knot, A is a 2x2
    // dense matrix.
    std::size_t numberOfExpectedTriplets = 4;
    if (numberOfInteriorKnots > 1)
    {
        constexpr std::size_t numberOfExtremumElements = 4;
        constexpr std::size_t numberOfNonExtremumElements = 6;
        constexpr std::size_t numberOfExtremum = 2;
        numberOfExpectedTriplets
            = 2
              * (numberOfNonExtremumElements * (numberOfInteriorKnots - numberOfExtremum)
                 + numberOfExtremum * numberOfExtremumElements);
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
            this->addTripletCurrentKnot(absoluteKnotIndex, i * 2, 0, tripletsList);
            this->addTripletNextKnot(absoluteKnotIndex, i * 2, 2, tripletsList);
        } else if (i + 1 == numberOfInteriorKnots)
        {
            this->addTripletPreviousKnot(absoluteKnotIndex, i * 2, 2 * (i - 1), tripletsList);
            this->addTripletCurrentKnot(absoluteKnotIndex, i * 2, 2 * (i - 1) + 2, tripletsList);
        } else
        {
            this->addTripletPreviousKnot(absoluteKnotIndex, i * 2, 2 * (i - 1), tripletsList);
            this->addTripletCurrentKnot(absoluteKnotIndex, i * 2, 2 * (i - 1) + 2, tripletsList);
            this->addTripletNextKnot(absoluteKnotIndex, i * 2, 2 * (i - 1) + 2 + 2, tripletsList);
        }
    }

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
                this->addKnownTermKnotPosition(absoluteKnotIndex, j, b.template head<2>());
                this->addKnownTermNextKnot(absoluteKnotIndex, j, b.template head<2>());
                this->addKnownTermPreviousKnot(absoluteKnotIndex, j, b.template head<2>());

            } else if (i == 0)
            {
                this->addKnownTermKnotPosition(absoluteKnotIndex, j, b.template head<2>());
                this->addKnownTermPreviousKnot(absoluteKnotIndex, j, b.template head<2>());

            } else if (i + 1 == numberOfInteriorKnots)
            {
                this->addKnownTermNextKnot(absoluteKnotIndex, j, b.template segment<2>(i * 2));
                this->addKnownTermKnotPosition(absoluteKnotIndex, j, b.template segment<2>(i * 2));
            } else
            {
                this->addKnownTermKnotPosition(absoluteKnotIndex, j, b.template segment<2>(i * 2));
            }
        }

        using EigenSparse = Eigen::SparseMatrix<typename T::Scalar>;
        using Ordering = typename Eigen::COLAMDOrdering<typename EigenSparse::StorageIndex>;

        Eigen::SparseQR<EigenSparse, Ordering> qrDecomposition;
        qrDecomposition.compute(A);
        Eigen::Matrix<typename T::Scalar, Eigen::Dynamic, 1> solution = qrDecomposition.solve(b);

        for (size_t i = 0; i < numberOfInteriorKnots; i++)
        {
            const int absoluteKnotIndex = i + 1;

            this->m_knots[absoluteKnotIndex].velocity[j] = solution[i * 2];
            this->m_knots[absoluteKnotIndex].acceleration[j] = solution[i * 2 + 1];
        }
    }
}
} // namespace Math
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATH_QUINTIC_SPLINE_H
