/**
 * @file LinearSpline.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_MATH_LINEAR_SPLINE_H
#define BIPEDAL_LOCOMOTION_MATH_LINEAR_SPLINE_H

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
 * Linearspline implements a 1-st order polynomial spline in \$f\mathbb{R}^n\$f.
 * @note The spline is defined as a set of piecewise polynomial functions of degree 1 of the form
 * \f[
 * s(t) = a_0 + a_1 t
 * \f]
 * where \$t \in [0, T_i]\$ and \$T_i\$ is the duration of the i-th polynomial.
 */
template <typename T> class LinearSpline : public Spline<T>
{

private:
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
};

template <typename T>
void LinearSpline<T>::updatePolynomialCoefficients(typename Spline<T>::Polynomial& poly)
{
    const double d = std::chrono::duration<double>(poly.duration).count();

    const auto& x0 = poly.initialPoint->position;
    const auto& xT = poly.finalPoint->position;

    // resize the coefficients
    poly.coefficients.resize(2);
    poly.coefficients[0] = x0;
    poly.coefficients[1] = (xT - x0) / d;
}

template <typename T> void LinearSpline<T>::computeIntermediateQuantities()
{
    return;
}

} // namespace Math
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATH_LINEAR_SPLINE_H
