/**
 * @file ZeroOrderSpline.h
 * @authors Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_MATH_ZERO_ORDER_SPLINE_H
#define BIPEDAL_LOCOMOTION_MATH_ZERO_ORDER_SPLINE_H

#include <chrono>
#include <memory>
#include <vector>

#include <BipedalLocomotion/Math/Spline.h>

namespace BipedalLocomotion
{
namespace Math
{
/**
 * ZeroOrderSpline implements a 0 order polynomial spline in \f$\mathbb{R}^n\f$.
 * @note The spline is defined as a set of piecewise polynomial functions of degree 0 of the form
 * \f[
 * s_i(t) = a_i
 * \f]
 * where \f$t \in [0, T_i]\f$ and \f$T_i\f$ is the duration of the i-th polynomial.
 */
template <typename T> class ZeroOrderSpline : public Spline<T>
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
void ZeroOrderSpline<T>::updatePolynomialCoefficients(typename Spline<T>::Polynomial& poly)
{
    const auto& x0 = poly.initialPoint->position;

    // resize the coefficients
    poly.coefficients.resize(1);
    poly.coefficients[0] = x0;
}

template <typename T> void ZeroOrderSpline<T>::computeIntermediateQuantities()
{
    return;
}

} // namespace Math
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATH_ZERO_ORDER_SPLINE_H
