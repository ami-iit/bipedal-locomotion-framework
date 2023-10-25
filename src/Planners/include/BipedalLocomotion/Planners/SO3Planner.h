/**
 * @file SO3Planner.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_H

#include <chrono>
#include <manif/SO3.h>

#include <BipedalLocomotion/System/Source.h>

namespace BipedalLocomotion
{
namespace Planners
{

/**
 * Trivialization used in the SO3Planner
 */
enum class LieGroupTrivialization
{
    Left,
    Right
};

/**
 * SO3PlannerState contains the state of the planner. The velocity and the acceleration are
 * expressed in inertial/body frame depending on the Trivialization used by the planner.
 */
struct SO3PlannerState
{
    manif::SO3d rotation; /**< \f${}^{\mathcal{I}}R_{\mathcal{B}}\f$ */
    manif::SO3d::Tangent velocity; /**< Angular velocity expressed in inertial or body fixed frame
                                    * (accordingly to the trivialization used). */
    manif::SO3d::Tangent acceleration; /**< Angular acceleration expressed in inertial or body fixed
                                        * frame (accordingly to the trivialization used). */
};

/**
 * SO3Planner implements a minimum jerk trajectory planner for object belonging to SO(3). The
 * planner assumes initial and final angular acceleration and velocity proportional to the error
 * between the initial and final rotation. Where the error formulation depends on the trivialization
 * used. In details, for left trivialized - body frame
 * \f[
 * \epsilon = \text{log} \left(R_0^\top R_f \right)
 * \f]
 * for right trivialized - inertial frame
 * \f[
 * \epsilon = \text{log} \left(R_f^\top R_0 \right)
 * \f]
 * @note The generated velocity and acceleration are written in the inertial or in body-fixed frame
 * accordingly to the chosen trivialization.
 * @tparam chosen trivialization:
 * - The right trivialization planner generates a velocity and an acceleration expressed in the
 * inertial frame.
 * - The left trivialization planner generates a velocity and an acceleration expressed in the
 * body-fixed frame.
 */
template <LieGroupTrivialization trivialization>
class SO3Planner : public System::Source<SO3PlannerState>
{
    /** Initial rotation from the inertial frame to the body frame. Namely
     * \f${}^{\mathcal{I}}R_{\mathcal{B}}\f$ */
    manif::SO3d m_initialRotation{manif::SO3d::Identity()};

    /** Distance bewteen the initial rotation and the final one. The definition of the distance
     * depends on the chosen Trivialization */
    manif::SO3d::Tangent m_distance{manif::SO3d::Tangent::Zero()};

    std::chrono::nanoseconds m_T{std::chrono::nanoseconds::zero()}; /**< Trajectory duration in
                                                                       seconds */

    /** Time step of the advance interface. */
    std::chrono::nanoseconds m_advanceTimeStep{std::chrono::nanoseconds::zero()};
    /** Current time of the advance object. */
    std::chrono::nanoseconds m_advanceCurrentTime{std::chrono::nanoseconds::zero()};

    SO3PlannerState m_state; /**< Current state of the planner. It is used by the advance
                                capabilities. */

    /**
     * Description of a 5-th polynomial. It contains the coefficients of the sub-trajectory.
     */
    struct Polynomial
    {
        double a0;
        double a1;
        double a2;
        double a3;
        double a4;
        double a5;
    };

    /**
     * Struct containing the boundary condition
     */
    struct BoundaryConditions
    {
        manif::SO3d::Tangent velocity;
        manif::SO3d::Tangent acceleration;
        bool isSet{false};
    };

    BoundaryConditions initialCondition; /**< Initial condition */
    BoundaryConditions finalCondition; /**< Final condition */
    bool areCoefficientsComputed{false}; /**< If true the coefficients are computed and updated */
    Polynomial polynomial;

    /**
     * This function is the entry point to compute the coefficients of the spline
     */
    bool computeCoefficients();

    /**
     * Compute the distance given two rotations. It depends on the trivialization.
     */
    static manif::SO3d::Tangent
    computeDistance(const manif::SO3d& initialRotation, const manif::SO3d& finalRotation);

public:
    /**
     * Set the rotation
     * @param initialRotation initial rotation \f${}^{\mathcal{I}}R_{\mathcal{B}_0}\f$.
     * @param finalRotation final rotation \f${}^{\mathcal{I}}R_{\mathcal{B}_T}\f$.
     * @param duration Trajectory duration in seconds.
     * @return True in case of success/false otherwise.
     */
    bool setRotations(const manif::SO3d& initialRotation,
                      const manif::SO3d& finalRotation,
                      const std::chrono::nanoseconds& duration);

    /**
     * Get the trajectory at a given time
     * @param time time, in seconds, at which the trajectory should be computed.
     * @param state state of the planner.
     * @return True in case of success/false otherwise.
     */
    bool evaluatePoint(const std::chrono::nanoseconds& time, SO3PlannerState& state);

    /**
     * Get the trajectory at a given time
     * @param time time, in seconds, at which the trajectory should be computed.
     * @param rotation \f${}^{\mathcal{I}}R_{\mathcal{B}}\f$.
     * @param velocity angular velocity expressed in inertial or body fixed frame (accordingly to
     * the trivialization used).
     * @param acceleration angular acceleration expressed in inertial or body fixed frame
     * (accordingly to the trivialization used).
     * @return True in case of success/false otherwise.
     */
    template <class Derived>
    bool evaluatePoint(const std::chrono::nanoseconds& time,
                       manif::SO3d& rotation,
                       manif::SO3TangentBase<Derived>& velocity,
                       manif::SO3TangentBase<Derived>& acceleration);

    /**
     * Set the time step of the advance interface.
     * @warning if the the time step is not set the user cannot use the advance features.
     * @param dt the time step of the advance block.
     * @return True in case of success, false otherwise.
     */
    bool setAdvanceTimeStep(const std::chrono::nanoseconds& dt);

    /**
     * Set the initial condition of the spline
     * @param velocity initial angular velocity.
     * @param acceleration initial angular acceleration.
     * @return True in case of success, false otherwise.
     */
    bool setInitialConditions(const manif::SO3d::Tangent& velocity,
                              const manif::SO3d::Tangent& acceleration);

    /**
     * Set the final condition of the trajectory generator
     * @param velocity final angular velocity.
     * @param acceleration final angular acceleration.
     * @return True in case of success, false otherwise.
     */
    bool setFinalConditions(const manif::SO3d::Tangent& velocity,
                            const manif::SO3d::Tangent& acceleration);

    /**
     * Get the state of the system.
     * @warning if the the time step of the advance is not set the user cannot use the advance
     * features.
     * @return a const reference of the requested object.
     */
    const SO3PlannerState& getOutput() const final;

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

    /**
     * Project the tangent vector on the vector that represents the distance between the two
     * rotations.
     * @param initialRotation initial rotation \f${}^{\mathcal{I}}R_{\mathcal{B}_0}\f$.
     * @param finalRotation final rotation \f${}^{\mathcal{I}}R_{\mathcal{B}_T}\f$.
     * @param vector Tangent vector that needs to be projected.
     * @return The projected tangent vector.
     * @warning If the initial and final rotations coincides the projected tangent vector will be
     * set equal to zero.
     * @note this method can be used along with setInitialConditions and setFinalConditions.
     */
    static manif::SO3d::Tangent projectTangentVector(const manif::SO3d& initialRotation,
                                                     const manif::SO3d& finalRotation,
                                                     const manif::SO3d::Tangent& vector);
};

/**
 * The right trivialization planner generates a velocity and an acceleration expressed in the
 * inertial frame.
 */
using SO3PlannerInertial = SO3Planner<LieGroupTrivialization::Right>;

/**
 * The left trivialization planner generates a velocity and an acceleration expressed in the
 * body-fixed frame.
 */
using SO3PlannerBody = SO3Planner<LieGroupTrivialization::Left>;

} // namespace Planners
} // namespace BipedalLocomotion

#include <BipedalLocomotion/Planners/SO3Planner.tpp>

#endif // BIPEDAL_LOCOMOTION_PLANNERS_SO3_PLANNER_H
