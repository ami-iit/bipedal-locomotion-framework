/**
 * @file IntegrationBasedIK.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_INTEGRATION_BASE_IK_H
#define BIPEDAL_LOCOMOTION_IK_INTEGRATION_BASE_IK_H

#include <unordered_map>
#include <type_traits>

#include <Eigen/Dense>
#include <manif/SE3.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/System/ILinearTaskSolver.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/System/WeightProvider.h>

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
 * velocity. System::FloatingBaseSystemVelocityKinematics and System::Integrator classes can be used
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

/**
 * IntegrationBasedIKProblem store all the ingredients to run an modfy at runtime an
 * Integration based inverse kinematics problem.
 */
struct IntegrationBasedIKProblem
{
    System::VariablesHandler variablesHandler; /**< Container of the variables considered by the IK
                                                * problem
                                                */
    /** Map containing the weight associated to each task stored considering the task name */
    std::unordered_map<std::string, std::shared_ptr<System::WeightProvider>> weights;
    std::unique_ptr<IntegrationBasedIK> ik; /**< Pointer to the IK solver */

    /**
     * Check if the problem is valid.
     * @return true if the problem is valid.
     */
    [[nodiscard]] bool isValid() const;

    /**
     * Get the element of the problem from a given index.
     * @tparam index a positive number from 0 to 2.
     * @note 0 is associated to the variablesHandler, 1 to the weights and 2 to the ik. Thanks to
     * this method the IntegrationBasedIKProblem behaves as a tuple. I.e., the
     * [`std::tie()`](https://en.cppreference.com/w/cpp/utility/tuple/tie) or
     * [Structured binding declaration](https://en.cppreference.com/w/cpp/language/structured_binding)
     * are supported by IntegrationBasedIKProblem.
     * @note the implementation of this method was taken from
     * https://devblogs.microsoft.com/oldnewthing/20201015-00/?p=104369
     */
    template <std::size_t index> std::tuple_element_t<index, IntegrationBasedIKProblem>& get()
    {
        if constexpr (index == 0)
        {
            return variablesHandler;
        }
        if constexpr (index == 1)
        {
            return weights;
        }
        if constexpr (index == 2)
        {
            return ik;
        }
    }
};

} // namespace IK
} // namespace BipedalLocomotion

namespace std
{

// The following methods are required to make ::BipedalLocomotion::IK::IntegrationBasedIKProblem
// behaving as a tuple. Please check here:
// https://devblogs.microsoft.com/oldnewthing/20201015-00/?p=104369
template <>
struct tuple_size<::BipedalLocomotion::IK::IntegrationBasedIKProblem> : integral_constant<size_t, 3>
{
};

template <size_t Index>
struct tuple_element<Index, ::BipedalLocomotion::IK::IntegrationBasedIKProblem>
    : tuple_element<
          Index,
          tuple<::BipedalLocomotion::System::VariablesHandler,
                unordered_map<string, shared_ptr<::BipedalLocomotion::System::WeightProvider>>,
                unique_ptr<::BipedalLocomotion::IK::IntegrationBasedIK>>>
{
};

} // namespace std

#endif // BIPEDAL_LOCOMOTION_IK_INTEGRATION_BASE_INVERSE_KINEMATICS_H
