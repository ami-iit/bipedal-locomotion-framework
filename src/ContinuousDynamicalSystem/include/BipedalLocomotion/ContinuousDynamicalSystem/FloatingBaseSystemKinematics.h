/**
 * @file FloatingBaseSystemKinematics.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FLOATING_BASE_SYSTEM_KINEMATICS_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FLOATING_BASE_SYSTEM_KINEMATICS_H

#include <memory>
#include <vector>

#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

#include <Eigen/Dense>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

// Forward declare for type traits specialization
class FloatingBaseSystemKinematics;
}
}

// Please read it as
// BLF_DEFINE_CONTINUOUS_DYNAMICAL_SYSTEM_INTERAL_STRUCTURE(
//     FloatingBaseSystemKinematics,
//     (base position, base orientation, joint positions),
//     (base linear velocity, rate of change of base rotation matrix, joint velocities),
//     (base twist expressed in mixed representation, joint velocities))
BLF_DEFINE_CONTINUOUS_DYNAMICAL_SYSTEM_INTERAL_STRUCTURE(
    FloatingBaseSystemKinematics,
    (Eigen::Vector3d, Eigen::Matrix3d, Eigen::VectorXd),
    (Eigen::Vector3d, Eigen::Matrix3d, Eigen::VectorXd),
    (Eigen::Matrix<double, 6, 1>, Eigen::VectorXd))

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * FloatingBaseSystemKinematics describes a floating base system kinematics.
 * The FloatingBaseSystemKinematics inherits from a generic DynamicalSystem where:
 * - DynamicalSystem::State is described by an std::tuple containing:
 *   - Eigen::Vector3d: position of the base w.r.t. the inertial frame
 *   - Eigen::Matrix3d: rotation matrix \f${} ^ I R _ {b}\f$. Matrix that transform a vector
 * whose coordinates are expressed in the base frame in the inertial frame;
 *   - Eigen::VectorXd: the joint positions [in rad].
 * - DynamicalSystem::StateDerivative is described by an std::tuple containing:
 *   - Eigen::Vector3d: base linear velocity w.r.t. the inertial frame;
 *   - Eigen::Matrix3d: rate of change of the rotation matrix \f${} ^ I \dot{R} _ {b}\f$.
 * whose coordinates are expressed in the base frame in the inertial frame;
 *   - Eigen::VectorXd: the joint velocities [in rad/s].
 * - DynamicalSystem::Input is described by an std::tuple containing:
 *   - Eigen::Vector6d: base twist w.r.t. the inertial frame;
 *   - Eigen::VectorXd: the joint velocities [in rad/s].
 */
class FloatingBaseSystemKinematics : public DynamicalSystem<FloatingBaseSystemKinematics>
{
    double m_rho{0.01}; /**< Regularization term used for the Baumgarte stabilization over the SO(3)
                           group */

    State m_state;
    Input m_controlInput;

public:
    /**
     * Initialize the FloatingBaseSystemKinematics system.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are used
     * | Parameter Name |   Type   |                                   Description                                     | Mandatory |
     * |:--------------:|:--------:|:---------------------------------------------------------------------------------:|:---------:|
     * |      `rho`     | `double` | Baumgarte stabilization parameter over the SO(3) group. The default value is 0.01 |    No     |
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler);

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
    bool dynamics(const double& time, StateDerivative& stateDerivative);
};

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FLOATING_BASE_SYSTEM_KINEMATICS_H
