/**
 * @file FloatingBaseSystemKinematics.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FLOATING_BASE_SYSTEM_KINEMATICS_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FLOATING_BASE_SYSTEM_KINEMATICS_H

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
class FloatingBaseSystemKinematics;
}
}

namespace BipedalLocomotion::ContinuousDynamicalSystem::internal
{
template <> struct traits<FloatingBaseSystemKinematics>
{
    using Twist = Eigen::Matrix<double, 6, 1>;
    using State = GenericContainer::named_tuple<BLF_NAMED_PARAM(p, Eigen::Vector3d),
                                      BLF_NAMED_PARAM(R, manif::SO3d),
                                      BLF_NAMED_PARAM(s, Eigen::VectorXd)>;
    using StateDerivative = GenericContainer::named_tuple<BLF_NAMED_PARAM(dp, Eigen::Vector3d),
                                                BLF_NAMED_PARAM(omega, manif::SO3d::Tangent),
                                                BLF_NAMED_PARAM(ds, Eigen::VectorXd)>;
    using Input = GenericContainer::named_tuple<BLF_NAMED_PARAM(twist, Twist),
                                      BLF_NAMED_PARAM(ds, Eigen::VectorXd)>;
    using DynamicalSystem = FloatingBaseSystemKinematics;
};
} // namespace BipedalLocomotion::ContinuousDynamicalSystem::internal

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * FloatingBaseSystemKinematics describes a floating base system kinematics.
 * The FloatingBaseSystemKinematics inherits from a generic DynamicalSystem where the State is
 * described by a BipedalLocomotion::GenericContainer::named_tuple
 * | Name |        Type       |                                                                  Description                                                                 |
 * |:----:|:-----------------:|:--------------------------------------------------------------------------------------------------------------------------------------------:|
 * |  `p` | `Eigen::Vector3d` |                                                Position of the base w.r.t. the inertial frame                                                |
 * |  `R` |   `manif::SO3d`   | Rotation matrix \f${} ^ I R _ {b}\f$. Matrix that transform a vector whose coordinates are expressed in the base frame in the inertial frame |
 * |  `s` | `Eigen::VectorXd` |                                                           Joint positions [in rad]                                                           |
 *
 * The `StateDerivative` is described by a BipedalLocomotion::GenericContainer::named_tuple
 * |   Name  |          Type          |                                                         Description                                                         |
 * |:-------:|:----------------------:|:---------------------------------------------------------------------------------------------------------------------------:|
 * |  `dp`   |    `Eigen::Vector3d`   | Linear velocity of the origin of the base link whose coordinates are expressed in the Inertial frame (MIXED RERPESENTATION) |
 * | `omega` | `manif::SO3d::Tangent` |                base angular velocity whose coordinates are expressed in the inertial frame (Left trivialized)               |
 * |   `ds`  |    `Eigen::VectorXd`   |                                                 Joint velocities [in rad/s]                                                 |
 *
 * The `Input` is described by a BipedalLocomotion::GenericContainer::named_tuple
 * |   Name  |              Type             |                  Description                 |
 * |:-------:|:-----------------------------:|:--------------------------------------------:|
 * | `twist` | `Eigen::Matrix<double, 6, 1>` | Base twist expressed in mixed representation |
 * |   `ds`  |       `Eigen::VectorXd`       |          Joint velocities [in rad/s]         |
 */
class FloatingBaseSystemKinematics : public DynamicalSystem<FloatingBaseSystemKinematics>
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

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FLOATING_BASE_SYSTEM_KINEMATICS_H
