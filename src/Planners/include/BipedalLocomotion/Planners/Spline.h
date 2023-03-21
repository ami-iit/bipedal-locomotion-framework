/**
 * @file Spline.h
 * @authors Giulio Romualdi
 * @copyright 2020. 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_SPLINE_H
#define BIPEDAL_LOCOMOTION_PLANNERS_SPLINE_H

#include <chrono>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <BipedalLocomotion/System/Source.h>

namespace BipedalLocomotion
{
namespace Planners
{

struct SplineState
{
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd acceleration;
};

/**
 * spline implement an interface for a Spline in \f$\mathbb{R}^n\f$
 */
class Spline : public System::Source<SplineState>
{
public:
    /**
     * Destructor.
     */
    virtual ~Spline() = default;

    /**
     * Set the time step of the advance interface.
     * @warning if the the time step is not set the user cannot use the advance features.
     * @param dt the time step of the advance block.
     * @return True in case of success, false otherwise.
     */
    virtual bool setAdvanceTimeStep(const std::chrono::nanoseconds& dt) = 0;

    /**
     * Set the knots of the spline.
     * @param position position of the knots in \$f\mathbb{R}^n\$f.
     * @param time vector containing the time instant of the knots.
     * @return True in case of success, false otherwise.
     */
    virtual bool setKnots(const std::vector<Eigen::VectorXd>& position, //
                          const std::vector<std::chrono::nanoseconds>& time)
        = 0;

    /**
     * Set the initial condition of the spline
     * @param velocity initial velocity (i.e. first derivative).
     * @param acceleration initial acceleration (i.e. second derivative).
     * @return True in case of success, false otherwise.
     */
    virtual bool setInitialConditions(Eigen::Ref<const Eigen::VectorXd> velocity,
                                      Eigen::Ref<const Eigen::VectorXd> acceleration)
        = 0;

    /**
     * Set the final condition of the spline
     * @param velocity final velocity (i.e. first derivative).
     * @param acceleration final acceleration (i.e. second derivative).
     * @return True in case of success, false otherwise.
     */
    virtual bool setFinalConditions(Eigen::Ref<const Eigen::VectorXd> velocity,
                                    Eigen::Ref<const Eigen::VectorXd> acceleration)
        = 0;

    /**
     * Evaluate the spline at a given point
     * @param t instant time
     * @param position position at time t
     * @param velocity velocity at time t
     * @param acceleration acceleration at time t
     * @return True in case of success, false otherwise.
     */
    virtual bool evaluatePoint(const std::chrono::nanoseconds& t,
                               Eigen::Ref<Eigen::VectorXd> position,
                               Eigen::Ref<Eigen::VectorXd> velocity,
                               Eigen::Ref<Eigen::VectorXd> acceleration)
        = 0;

    /**
     * Evaluate the spline at a given point
     * @param t instant time
     * @param state of the system
     * @return True in case of success, false otherwise.
     */
    virtual bool evaluatePoint(const std::chrono::nanoseconds& t, SplineState& state) = 0;
};
} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_SPLINE_H
