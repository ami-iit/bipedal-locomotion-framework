/**
 * @file QuinticSpline.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_QUINTIC_SPLINE_H
#define BIPEDAL_LOCOMOTION_PLANNERS_QUINTIC_SPLINE_H

#include <vector>
#include <memory>

#include <Eigen/Dense>

namespace BipedalLocomotion
{
namespace Planners
{
/**
 * Quintic spline implement a 5-th order polynomial spline in \$f\mathbb{R}^n\$f.
 */
class QuinticSpline
{
    /**
     * Private implementation of the class
     */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl; /**< Private implementation */

public:

    /**
     * Constructor.
     */
    QuinticSpline();

    /**
     * Destructor.
     * @note It is required by the PIMPL idiom.
     */
    ~QuinticSpline();

    /**
     * Set the knots of the spline.
     * @param position position of the knots in \$f\mathbb{R}^n\$f.
     * @param time vector containing the time instant of the knots.
     * @return True in case of success, false otherwise.
     */
    bool setKnots(const std::vector<Eigen::VectorXd>& position, const std::vector<double>& time);

    /**
     * Set the initial condition of the spline
     * @param velocity initial velocity (i.e. first derivative).
     * @param acceleration initial acceleration (i.e. second derivative).
     * @return True in case of success, false otherwise.
     */
    bool setInitialConditions(Eigen::Ref<const Eigen::VectorXd> velocity,
                              Eigen::Ref<const Eigen::VectorXd> acceleration);

    /**
     * Set the final condition of the spline
     * @param velocity final velocity (i.e. first derivative).
     * @param acceleration final acceleration (i.e. second derivative).
     * @return True in case of success, false otherwise.
     */
    bool setFinalConditions(Eigen::Ref<const Eigen::VectorXd> velocity,
                            Eigen::Ref<const Eigen::VectorXd> acceleration);

    /**
     * Evaluate the spline at a given point
     * @param t instant time
     * @param position position at time t
     * @param velocity velocity at time t
     * @param acceleration acceleration at time t
     * @return True in case of success, false otherwise.
     */
    bool evaluatePoint(const double& t,
                       Eigen::Ref<Eigen::VectorXd> position,
                       Eigen::Ref<Eigen::VectorXd> velocity,
                       Eigen::Ref<Eigen::VectorXd> acceleration);
};
} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_QUINTIC_SPLINE_H
