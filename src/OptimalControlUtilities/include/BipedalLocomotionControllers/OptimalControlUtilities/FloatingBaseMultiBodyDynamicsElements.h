/**
 * @file FloatingBaseMultiBodyDynamicsElement.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_FLOATING_BASE_MULTI_BODY_DYNAMICS_ELEMENT_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_FLOATING_BASE_MULTI_BODY_DYNAMICS_ELEMENT_H

#include <iDynTree/Model/FreeFloatingState.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>

namespace BipedalLocomotionControllers
{

namespace OptimalControlUtilities
{

/**
 * FloatingBaseMultiBodyDynamicsElement describes the System Dynamics that will be embedded as
 * equality constraint or cost function.
 */
class FloatingBaseMultiBodyDynamicsElement : public ControlTask
{
    /** Index range of the base acceleration */
    iDynTree::IndexRange m_baseAccelerationIndex{iDynTree::IndexRange::InvalidRange()};

    /** Index range of the joint acceleration */
    iDynTree::IndexRange m_jointAccelerationIndex{iDynTree::IndexRange::InvalidRange()};

    /** Index range of the joint torque */
    iDynTree::IndexRange m_jointTorqueIndex{iDynTree::IndexRange::InvalidRange()};

    iDynTree::MatrixDynSize m_massMatrix; /**< Floating-base mass matrix  */
    iDynTree::FreeFloatingGeneralizedTorques m_generalizedBiasForces; /**< Coriolis and
                                                                         Gravitational term  */
    iDynTree::MatrixDynSize m_jacobianMatrix; /**< Jacobian Matrix  */

    std::vector<Frame> m_framesInContact; /**< Vectors containing the frames in contact with the
                                             environment */

    iDynTree::MatrixDynSize m_reflectedInertia; /**< Reflected inertia matrix  */
    bool m_useReflectedInertia; /**< If true the reflected inertia will be used  */

public:
    /**
     * Constructor. If you call this constructor the motor reflected inertia is disabled
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact.
     * @throw std::runtime_error if the frame is not defined
     */
    FloatingBaseMultiBodyDynamicsElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                         const VariableHandler& handler,
                                         const std::vector<FrameNames>& framesInContact);

    /**
     * Constructor. If you call this constructor the motor reflected inertia is enable
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact.
     * @param regularizationMatrix joints regularization mass matrix. It can be used to handle
     * the motor reflected inertia
     * @throw std::runtime_error if the frame is not defined or the regularizationMatrix size is
     * no coherent with the number of joints
     */
    FloatingBaseMultiBodyDynamicsElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                         const VariableHandler& handler,
                                         const std::vector<FrameNames>& framesInContact,
                                         const iDynTree::MatrixDynSize& regularizationMatrix);

    /**
     * Get (and compute) the element matrix
     * @return the element matrix
     */
    virtual const iDynTree::MatrixDynSize& getA() final;

    /**
     * Get (and compute) the element vector
     * @return the element vector
     */
    virtual const iDynTree::VectorDynSize& getB() final;
};

} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_FLOATING_BASE_MULTI_BODY_DYNAMICS_ELEMENT_H
