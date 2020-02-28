/**
 * @file FloatingBaseMultiBodyDynamicsElements.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_FLOATING_BASE_MULTI_BODY_DYNAMICS_ELEMENT_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_FLOATING_BASE_MULTI_BODY_DYNAMICS_ELEMENT_H

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Model/FreeFloatingState.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>

namespace BipedalLocomotionControllers
{

namespace OptimalControlUtilities
{

/**
 * MultiBodyDynamicsElement describes the dynamics of the floating base system.
 * Given a floating based system dynamics \f$ M \dot{\nu} + h = J^\top F + S \tau \f$ the matrices
 * and vectors can be factorized as follows:
 * \f[
 * M = \begin{bmatrix}
 * M_{b} & M_{bs} \\ M_{sb} & M_{s}
 * \end{bmatrix} \quad
 * h = \begin{bmatrix}
 * h_{b} \\  h_{s}
 * \end{bmatrix} \quad
 * J^\top = \begin{bmatrix}
 * J^\top_{b} \\ J^\top_{s}
 * \end{bmatrix}
 * \f]
 * So the dynamics of the system can be written as
  * \f[
  \begin{bmatrix}
 * M_{b} & M_{bs} \\ M_{sb} & M_{s}
 * \end{bmatrix}
 * \begin{bmatrix}
 * \dot{\nu}_{b} \\ \ddot{s}
 * \end{bmatrix} +
 * \begin{bmatrix}
 * h_{b} \\ h_{s}
 * \end{bmatrix} =
 * \begin{bmatrix}
 * 0 \\ I
 * \end{bmatrix} \tau +
 * \begin{bmatrix}
 * J^\top_{b} \\
 * J^\top_{s}
 * \end{bmatrix} F
 * \f]
 */
class MultiBodyDynamicsElement : public ControlTask
{
protected:

    /** Index range of the joint acceleration */
    iDynTree::IndexRange m_jointAccelerationIndex{iDynTree::IndexRange::InvalidRange()};

    /** Index range of the base acceleration */
    iDynTree::IndexRange m_baseAccelerationIndex{iDynTree::IndexRange::InvalidRange()};

    iDynTree::MatrixDynSize m_massMatrix; /**< Floating-base mass matrix  */
    iDynTree::FreeFloatingGeneralizedTorques m_generalizedBiasForces; /**< Coriolis and
                                                                         Gravitational term  */
    iDynTree::MatrixDynSize m_jacobianMatrix; /**< Jacobian Matrix  */

    /** Map containing the frames in contact with the environment, the key is the name of the frame,
     * while the value is a structure describing a frame in contact with the  environment */
    std::unordered_map<std::string, FrameInContactWithWrench<iDynTree::IndexRange, iDynTree::FrameIndex>> m_framesInContact;

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact.
     * @throw std::runtime_error if the frame is not defined
     */
    MultiBodyDynamicsElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                             const VariableHandler& handler,
                             const std::vector<FrameInContact<std::string, std::string>>& framesInContact);

    /**
     */
    bool setMeasuredContactWrenches(const std::unordered_map<std::string, iDynTree::Wrench>& contactWrenches);

    /**
     * Set the model used to describe the contact. The current version of the library considers
     * compliant and stiff contacts
     * @param frameName name of the frame associated to the link
     * @param isCompliant true if the contact between the link associated to the frame and the
     * environment is compliant, false if it is considered stiff
     */
    void setCompliantContact(const std::string& frameName, bool isCompliant);
};

/**
 * FloatingBaseDynamicsElement describes <b>only</b> the dynamics of the floating base system.
 * Using the notation presented in @ref
 * BipedalLocomotionControllers::OptimalControlUtilities::MultiBodyDynamicsElement the
 * FloatingBaseDynamicsElement will consider only the first 6 rows of the system dynamics
 */
class FloatingBaseDynamicsElement : public MultiBodyDynamicsElement
{
public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact.
     * @throw std::runtime_error if the frame is not defined
     */
    FloatingBaseDynamicsElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                const VariableHandler& handler,
                                const std::vector<FrameInContact<std::string, std::string>>& framesInContact);

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

/**
 * JointSpaceDynamicsElement describes <b>only</b> the dynamics of the joints.
 * Using the notation presented in @ref
 * BipedalLocomotionControllers::OptimalControlUtilities::MultiBodyDynamicsElement the
 * JointSpaceDynamicsElement will consider only the last \a n rows of the system dynamics, where \a
 * n is the number of actuated DoFs.
 */
class JointSpaceDynamicsElement : public MultiBodyDynamicsElement
{
    /** Index range of the joint torque */
    iDynTree::IndexRange m_jointTorqueIndex{iDynTree::IndexRange::InvalidRange()};

    iDynTree::MatrixDynSize m_reflectedInertia; /**< Reflected inertia matrix  */
    bool m_useReflectedInertia{false}; /**< If true the reflected inertia will be used  */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact.
     * @throw std::runtime_error if the frame is not defined
     */
    JointSpaceDynamicsElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                              const VariableHandler& handler,
                              const std::vector<FrameInContact<std::string, std::string>>& framesInContact);

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
    JointSpaceDynamicsElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                              const VariableHandler& handler,
                              const std::vector<FrameInContact<std::string, std::string>>& framesInContact,
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

/**
 * WholeBodyFloatingBaseDynamicsElement describes the <b>entire</b> dynamics of the system,
 * considering both base and joint dynamics. Using the notation presented in @ref
 * BipedalLocomotionControllers::OptimalControlUtilities::MultiBodyDynamicsElement the
 * JointSpaceDynamicsElement will consider the entire equation.
 */
class WholeBodyFloatingBaseDynamicsElement : public MultiBodyDynamicsElement
{
    /** Index range of the joint torque */
    iDynTree::IndexRange m_jointTorqueIndex{iDynTree::IndexRange::InvalidRange()};

    iDynTree::MatrixDynSize m_reflectedInertia; /**< Reflected inertia matrix  */
    bool m_useReflectedInertia{false}; /**< If true the reflected inertia will be used  */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact.
     * @throw std::runtime_error if the frame is not defined
     */
    WholeBodyFloatingBaseDynamicsElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                         const VariableHandler& handler,
                                         const std::vector<FrameInContact<std::string, std::string>>& framesInContact);

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
    WholeBodyFloatingBaseDynamicsElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                         const VariableHandler& handler,
                                         const std::vector<FrameInContact<std::string, std::string>>& framesInContact,
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
