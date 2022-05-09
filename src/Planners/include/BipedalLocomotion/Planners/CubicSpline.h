/**
 * @file CubicSpline.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_CUBIC_SPLINE_H
#define BIPEDAL_LOCOMOTION_PLANNERS_CUBIC_SPLINE_H

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <BipedalLocomotion/Planners/Spline.h>

namespace BipedalLocomotion
{
namespace Planners
{
/**
 * Cubic spline implement a 3-rd order polynomial spline in \$f\mathbb{R}^n\$f.
 */
class CubicSpline : public Spline
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
    CubicSpline();

    /**
     * Destructor.
     * @note It is required by the PIMPL idiom.
     */
    ~CubicSpline();

    /**
     * Set the time step of the advance interface.
     * @warning if the the time step is not set the user cannot use the advance features.
     * @param dt the time step of the advance block.
     * @return True in case of success, false otherwise.
     */
    bool setAdvanceTimeStep(const double& dt) final;

    /**
     * Set the knots of the spline.
     * @param position position of the knots in \$f\mathbb{R}^n\$f.
     * @param time vector containing the time instant of the knots.
     * @return True in case of success, false otherwise.
     */
    bool setKnots(const std::vector<Eigen::VectorXd>& position, //
                  const std::vector<double>& time) final;

    /**
     * Set the initial condition of the spline
     * @param velocity initial velocity (i.e. first derivative).
     * @param acceleration initial acceleration (i.e. second derivative).
     * @note the acceleration is not considered in the spline evaluation
     * @return True in case of success, false otherwise.
     */
    bool setInitialConditions(Eigen::Ref<const Eigen::VectorXd> velocity,
                              Eigen::Ref<const Eigen::VectorXd> acceleration) final;

    /**
     * Set the final condition of the spline
     * @param velocity final velocity (i.e. first derivative).
     * @param acceleration final acceleration (i.e. second derivative).
     * @note the acceleration is not considered in the spline evaluation
     * @return True in case of success, false otherwise.
     */
    bool setFinalConditions(Eigen::Ref<const Eigen::VectorXd> velocity,
                            Eigen::Ref<const Eigen::VectorXd> acceleration) final;

    /**
     * Set the initial condition of the spline
     * @param velocity initial velocity (i.e. first derivative).
     * @return True in case of success, false otherwise.
     */
    bool setInitialConditions(Eigen::Ref<const Eigen::VectorXd> velocity);

    /**
     * Set the final condition of the spline
     * @param velocity final velocity (i.e. first derivative).
     * @return True in case of success, false otherwise.
     */
    bool setFinalConditions(Eigen::Ref<const Eigen::VectorXd> velocity);

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
                       Eigen::Ref<Eigen::VectorXd> acceleration) final;

    /**
     * Evaluate the spline at a given point
     * @param t instant time
     * @param state of the system
     * @return True in case of success, false otherwise.
     */
    bool evaluatePoint(const double& t, SplineState& state) final;

    /**
     * Get the state of the system.
     * @warning if the the time step of the advance is not set the user cannot use the advance
     * features.
     * @return a const reference of the requested object.
     */
    const SplineState& getOutput() const final;

    /**
     * Determines the validity of the object retrieved with get()
     * @warning if the the time step of the advance is not set the user cannot use the advance
     * features.
     * @return True if the object is valid, false otherwise.
     */
    bool isOutputValid() const final;

    /**
     * Advance the internal state. This may change the value retrievable from get().
     * @warning if the the time step of the advance is not set the user cannot use the advance
     * features.
     * @return True if the advance is successfull.
     */
    bool advance() final;
};
} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_CUBIC_SPLINE_H
