/**
 * @file IntegrationBasedIK.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_INTEGRATION_BASE_IK_H
#define BIPEDAL_LOCOMOTION_IK_INTEGRATION_BASE_IK_H

#include <Eigen/Dense>
#include <manif/SE3.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/System/ILinearTaskSolver.h>

namespace BipedalLocomotion
{

namespace IK
{

/**
 * State of the IntegrationBasedIK
 */
struct IntegrationBasedIKState
{
    Eigen::VectorXd jointVelocity; /**< Joints velocity in rad per seconds */
    manif::SE3d::Tangent baseVelocity; /**< Mixed spatial velocity of the base */
};

/**
 * IntegrationBasedIK implements the interface for the integration base inverse
 * kinematics. Please inherits this class if you want to implement your custom Integration base
 * Inverse Kinematics. The IntegrationBasedInverseKinematics can actually be used as Velocity
 * controller or real IK. Indeed it is important to notice that IntegrationBasedIKState is a struct
 * containing the joint velocities. When a robot velocity controller is available, one can set these
 * joint velocities to the low-level robot controller. In this case, the \f$t ^ d\f$ quantities in
 * the following figures  can be evaluated by using robot sensor feedback, and the robot is said to
 * be velocity controlled. On the other hand, if the robot velocity control is not available, one
 * may integrate the outcome of IntegrationBasedIK to obtain the desired joint position to be set to
 * a low-level robot position controller. In this case, the \f$t ^d\f$ quantities can be evaluated
 * by using the desired integrated quantities instead of sensor feedback, and the block behaves as
 * an inverse kinematics module, and the robot is said to be position controlled.
 * @subsection vc Velocity Control
 * Here you can find an example of the IntegrationBasedInverseKinematics interface used as
 * a velocity controller.
 * <br/>
 * <img src="https://user-images.githubusercontent.com/16744101/142453785-9e6f2b5e-dc82-417a-a5e3-bc8c61865d0b.png" alt="VelocityControl" width="1500">
 * @subsection ik Inverse Kinematics
 * If you want to use IntegrationBasedInverseKinematics as IK you need to integrate the output
 * velocity. System::FloatingBaseSystemKinematics and System::Integrator classes can be used
 * to integrate the output of the IK taking into account the geometrical structure of the
 * configuration space (\f$ \mathbb{R}^3 \times SO(3) \times \mathbb{R}^n\f$)
 * <br/>
 * <img src="https://user-images.githubusercontent.com/16744101/142453860-6bba2a7a-26af-48da-b04e-114314c6f67c.png" alt="InverseKinematics" width="1500">
 */
class IntegrationBasedIK : public System::ILinearTaskSolver<IKLinearTask, IntegrationBasedIKState>
{
public:
    /**
     * Destructor.
     */
    virtual ~IntegrationBasedIK() = default;
};
} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_IK_INTEGRATION_BASE_INVERSE_KINEMATICS_H
