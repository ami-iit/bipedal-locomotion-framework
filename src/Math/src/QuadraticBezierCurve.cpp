/**
 * @file QuadraticBezierCurve.cpp
 * @authors Paolo Viceconte, Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Math/QuadraticBezierCurve.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::Math;

bool QuadraticBezierCurve::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[QuadraticBezierCurve::initialize]";
    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return false;
    }

    auto linspace = [](double a, double b, int N) {
        const double h = (b - a) / static_cast<double>(N - 1);
        std::vector<double> xs(N);
        typename std::vector<double>::iterator x;
        double val;
        for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        {
            *x = val;
        }
        return xs;
    };

    int steps;
    if (!ptr->getParameter("number_of_knots", steps) || steps <= 0)
    {
        log()->error("{} Unable to find a valid 'number_of_knots'. Please remember that should be "
                     "positive number.",
                     logPrefix);
        return false;
    }

    // evaluate the position
    m_knots = linspace(0, 1, steps);
    m_curve.resize(2, steps);

    return true;
}

Eigen::Ref<const Eigen::Matrix2Xd>
QuadraticBezierCurve::evaluateCurve(Eigen::Ref<const Eigen::Vector2d> initialPoint,
                                    Eigen::Ref<const Eigen::Vector2d> controlPoint,
                                    Eigen::Ref<const Eigen::Vector2d> finalPoint)
{
    for (int i = 0; i < m_knots.size(); i++)
    {
        const double knot = m_knots[i];
        m_curve.col(i).noalias() = (1 - knot) * (1 - knot) * initialPoint;
        m_curve.col(i).noalias() += 2 * knot * (1 - knot) * controlPoint;
        m_curve.col(i).noalias() += knot * knot * finalPoint;
    }

    return m_curve;
}
