/**
 * @file QuadraticBezierCurve.h
 * @authors Paolo Viceconte, Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_MATH_QUADRATIC_BEZIER_CURVE_H
#define BIPEDAL_LOCOMOTION_MATH_QUADRATIC_BEZIER_CURVE_H

#include <vector>

#include <Eigen/Dense>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace Math
{

/**
 * QuadraticBezierCurve implements a quadratic Bézier curve.
 * A quadratic Bézier curve is characterized by a group of control points P0 to P2. The initial and
 * final control points consistently represent the endpoints of the curve, while the intermediate
 * control point typically do not lie precisely on the curve. In the subsequent sections, the sums
 * should be interpreted as affine combinations, meaning that the coefficients add up to 1.
 * In detail QuadraticBezierCurve implements
 * \f[
 * x(t) = (1 - t)^2 P_0 + 2(1 - t)t P_1 + t^2 P_2
 * \f]
 * where \f$P_0\f$ \f$P_2\f$ are the initial and final point of the curve, while \f$P_1\f$
 * is the control point.
 */
class QuadraticBezierCurve
{
public:
    /**
     * Initialize the QuadraticBezierCurve
     * @param handler pointer to the parameter handler.
     * @note The following parameters are optional:
     * |  Parameter Name  |  Type |                  Description                  | Mandatory |
     * |:----------------:|:-----:|:---------------------------------------------:|:---------:|
     * |`number_of_knots` | `int` | Number of knots in the quadratic bézier curve |    Yes    |
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);

    /**
     * Evaluate the curve given in initial, control and final points.
     * @param initialPoint initial point of the curve.
     * @param controlPoint control point of the curve.
     * @param finalPoint final point of the curve.
     * @return a matrix containing the curve.
     */
    Eigen::Ref<const Eigen::Matrix2Xd> evaluateCurve(Eigen::Ref<const Eigen::Vector2d> initialPoint,
                                                     Eigen::Ref<const Eigen::Vector2d> controlPoint,
                                                     Eigen::Ref<const Eigen::Vector2d> finalPoint);

private:
    std::vector<double> m_knots; /**< Knots of the curve generated as linspace. */
    Eigen::Matrix2Xd m_curve; /**< Matrix containing the quadratic Bézier curve. */
};

} // namespace Math
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATH_QUADRATIC_BEZIER_CURVE_H
