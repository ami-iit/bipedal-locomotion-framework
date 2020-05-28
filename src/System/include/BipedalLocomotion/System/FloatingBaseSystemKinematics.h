/**
 * @file FloatingBaseSystemKinematics.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_FLOATING_BASE_SYSTEM_KINEMATICS_H
#define BIPEDAL_LOCOMOTION_SYSTEM_FLOATING_BASE_SYSTEM_KINEMATICS_H

#include <memory>
#include <vector>

#include <BipedalLocomotion/System/DynamicalSystem.h>
#include <BipedalLocomotion/System/ContactWrench.h>

#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>

namespace BipedalLocomotion
{
namespace System
{

/**
 * FloatingBaseSystemKinematics describes a floating base system kinematics.
 * The FloatingBaseSystemKinematics inherits from a generic DynamicalSystem where:
 * - DynamicalSystem::StateType is described by an std::tuple containing:
 *   - iDynTree::Position: position of the base w.r.t. the inertial frame
 *   - iDynTree::Rotation: rotation matrix \f${} ^ I R _ {b}\f$. Matrix that transform a vector
 * whose coordinates are expressed in the base frame in the inertial frame;
 *   - iDynTree::VectorDynsize: the joint positions [in rad].
 * - DynamicalSystem::StateDerivativeType is described by an std::tuple containing:
 *   - iDynTree::Vector3: base velocity w.r.t. the inertial frame;
 *   - iDynTree::Matrix3x3: rate of change of the rotation matrix \f${} ^ I \dot{R} _ {b}\f$.
 * whose coordinates are expressed in the base frame in the inertial frame;
 *   - iDynTree::VectorDynsize: the joint velocities [in rad/s].
 * - DynamicalSystem::InputType is described by an std::tuple containing:
 *   - iDynTree::Twist: base twist w.r.t. the inertial frame;
 *   - iDynTree::VectorDynsize: the joint velocities [in rad/s].
 */
class FloatingBaseSystemKinematics : public DynamicalSystem<std::tuple<iDynTree::Position,
                                                                       iDynTree::Rotation,
                                                                       iDynTree::VectorDynSize>,
                                                            std::tuple<iDynTree::Vector3,
                                                                       iDynTree::Matrix3x3,
                                                                       iDynTree::VectorDynSize>,
                                                            std::tuple<iDynTree::Twist,
                                                                       iDynTree::VectorDynSize>>
{
public:

    /**
     * Computes the floating based system dynamics. It return \f$f(x, u, t)\f$.
     * @note The control input has to be set separately with the method setControlInput.
     * @param state tuple containing a const reference to the state elements.
     * @param time the time at witch the dynamics is computed.
     * @param stateDynamics tuple containing a reference to the element of the state derivative
     * @return true in case of success, false otherwise.
     */
    bool dynamics(const StateType& state,
                  const double& time,
                  StateDerivativeType& stateDerivative) final;

    /**
     * Destructor.
     */
    ~FloatingBaseSystemKinematics() = default;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_FLOATING_BASE_SYSTEM_KINEMATICS_H
