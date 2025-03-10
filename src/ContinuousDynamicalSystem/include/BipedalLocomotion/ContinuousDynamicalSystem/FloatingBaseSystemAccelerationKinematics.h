/**
 * @file FloatingBaseSystemAccelerationKinematics.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FLOATING_BASE_SYSTEM_ACCELERATION_KINEMATICS_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FLOATING_BASE_SYSTEM_ACCELERATION_KINEMATICS_H

#include <memory>
#include <vector>

#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/GenericContainer/NamedTuple.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

#include <manif/SO3.h>

#include <Eigen/Dense>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

// Forward declare for type traits specialization
class FloatingBaseSystemAccelerationKinematics;
} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

namespace BipedalLocomotion::ContinuousDynamicalSystem::internal
{
template <> struct traits<FloatingBaseSystemAccelerationKinematics>
{
    using Twist = Eigen::Matrix<double, 6, 1>;
    using State = GenericContainer::named_tuple<BLF_NAMED_PARAM(p, Eigen::Vector3d),
                                                BLF_NAMED_PARAM(R, manif::SO3d),
                                                BLF_NAMED_PARAM(s, Eigen::VectorXd),
                                                BLF_NAMED_PARAM(dp, Eigen::Vector3d),
                                                BLF_NAMED_PARAM(omega, manif::SO3d::Tangent),
                                                BLF_NAMED_PARAM(ds, Eigen::VectorXd)>;
    using StateDerivative
        = GenericContainer::named_tuple<BLF_NAMED_PARAM(dp, Eigen::Vector3d),
                                        BLF_NAMED_PARAM(omega, manif::SO3d::Tangent),
                                        BLF_NAMED_PARAM(ds, Eigen::VectorXd),
                                        BLF_NAMED_PARAM(ddp, Eigen::Vector3d),
                                        BLF_NAMED_PARAM(domega, manif::SO3d::Tangent),
                                        BLF_NAMED_PARAM(dds, Eigen::VectorXd)>;
    using Input = GenericContainer::named_tuple<BLF_NAMED_PARAM(dtwist, Twist),
                                                BLF_NAMED_PARAM(dds, Eigen::VectorXd)>;
    using DynamicalSystem = FloatingBaseSystemAccelerationKinematics;
};
} // namespace BipedalLocomotion::ContinuousDynamicalSystem::internal

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

// clang-format off
/**
 * FloatingBaseSystemAccelerationKinematics describes the kinematics of a floating base system
 * at the acceleration level. It inherits from a generic DynamicalSystem where:
 *
 * - The **State** is represented as a named tuple with the following parameters:
 *
 *   | Name    | Type                      | Description                                                                                                                |
 *   |:-------:|:-------------------------:|:--------------------------------------------------------------------------------------------------------------------------:|
 *   | `p`     | `Eigen::Vector3d`         | Base position expressed in the inertial frame.                                                                             |
 *   | `R`     | `manif::SO3d`             | Base orientation. The rotation matrix \( {}^I R_b \) transforms vectors expressed in the base frame to the inertial frame. |
 *   | `s`     | `Eigen::VectorXd`         | Joint positions (in radians).                                                                                              |
 *   | `dp`    | `Eigen::Vector3d`         | Base linear velocity (i.e., time derivative of `p`), expressed in the inertial frame. (mixed representation)               |
 *   | `omega` | `manif::SO3d::Tangent`    | Base angular velocity, expressed in the inertial frame (using left trivialization).                                        |
 *   | `ds`    | `Eigen::VectorXd`         | Joint velocities (in rad/s).                                                                                               |
 *
 * - The **StateDerivative** is represented as a named tuple with the following parameters:
 *
 *   | Name      | Type                      | Description                                                                                             |
 *   |:---------:|:-------------------------:|:-------------------------------------------------------------------------------------------------------:|
 *   | `dp`      | `Eigen::Vector3d`         | Time derivative of the base position (i.e., base linear velocity).                                      |
 *   | `omega`   | `manif::SO3d::Tangent`    | Time derivative of the base orientation (related to angular velocity).                                  |
 *   | `ds`      | `Eigen::VectorXd`         | Time derivative of the joint positions (i.e., joint velocities).                                        |
 *   | `ddp`     | `Eigen::Vector3d`         | Base linear acceleration (second time derivative of `p`).                                               |
 *   | `domega`  | `manif::SO3d::Tangent`    | Base angular acceleration (time derivative of `omega`).                                                 |
 *   | `dds`     | `Eigen::VectorXd`         | Joint accelerations (in rad/s²).                                                                        |
 *
 * - The **Input** to the system is represented as a named tuple with the following parameters:
 *
 *   | Name      | Type                                        | Description                                                                                     |
 *   |:---------:|:-------------------------------------------:|:-----------------------------------------------------------------------------------------------:|
 *   | `dtwist`  | `Twist` (e.g., `Eigen::Matrix<double, 6, 1>`) | Base twist in mixed representation (typically combining both linear and angular components).  |
 *   | `dds`     | `Eigen::VectorXd`                           | Joint accelerations (in rad/s²).                                                                |
 *
 * This formulation encapsulates both the instantaneous state (including positions and velocities)
 * and their time derivatives (accelerations), while employing Lie group representations (i.e., \( SO(3) \))
 * for handling rotations.
 */
// clang-format on
class FloatingBaseSystemAccelerationKinematics
    : public DynamicalSystem<FloatingBaseSystemAccelerationKinematics>
{
    State m_state;
    Input m_controlInput;

public:
    /**
     * Initialize the Dynamical system.
     * @param handler pointer to the parameter handler.
     * @return true in case of success/false otherwise.
     * @note This function does nothing but it is required for CRTP.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);

    /**
     * Set the state of the dynamical system.
     * @param state tuple containing a const reference to the state elements.
     * @return true in case of success, false otherwise.
     */
    bool setState(const State& state);

    /**
     * Get the state to the dynamical system.
     * @return the current state of the dynamical system
     */
    const State& getState() const;

    /**
     * Set the control input to the dynamical system.
     * @param controlInput the value of the control input used to compute the system dynamics.
     * @return true in case of success, false otherwise.
     */
    bool setControlInput(const Input& controlInput);

    /**
     * Computes the floating based system dynamics. It return \f$f(x, u, t)\f$.
     * @note The control input and the state have to be set separately with the methods
     * setControlInput and setState.
     * @param time the time at witch the dynamics is computed.
     * @param stateDynamics tuple containing a reference to the element of the state derivative
     * @return true in case of success, false otherwise.
     */
    bool dynamics(const std::chrono::nanoseconds& time, StateDerivative& stateDerivative);
};

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FLOATING_BASE_SYSTEM_ACCELERATION_KINEMATICS_H
