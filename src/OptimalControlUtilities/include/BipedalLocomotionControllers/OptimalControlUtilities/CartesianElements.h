/**
 * @file CartesianElements.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CARTESIAN_ELEMENT_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CARTESIAN_ELEMENT_H

#include <iDynTree/Core/VectorFixSize.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/PDController.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>

namespace BipedalLocomotionControllers
{

namespace OptimalControlUtilities
{

/** Cartesian Element Type */
enum class CartesianElementType
{
    POSE,
    POSITION,
    ORIENTATION
};

/** Name of the axis controlled in case of ONE_DIMENSION element is chosen */
enum class CartesianElementAxisName : size_t
{
    X = 0,
    Y = 1,
    Z = 2,
    ALL = 3
};

/**
 * CartesianElement describes a Cartesian element that will be embedded as
 * equality constraint or cost function.
 * The Cartesian element in useful to control the position/orientation/pose of a frame/CoM of the
 * robot.
 * @tparam type type of the Cartesian element
 * @tparam axisName name of the axis controlled. The default value is CartesianElementAxisName::ALL
 */
template <
    CartesianElementType type,
    CartesianElementAxisName axisName = CartesianElementAxisName::ALL>
class CartesianElement : public ControlTask
{
private:
     using ControllerType = typename std::conditional_t<type == CartesianElementType::ORIENTATION,
                                                        OrientationPD,
                                                        std::conditional_t<type == CartesianElementType::POSITION && axisName == CartesianElementAxisName::ALL,
                                                                           LinearPD<iDynTree::Vector3>,
                                                                           std::conditional_t<type == CartesianElementType::POSITION && axisName != CartesianElementAxisName::ALL,
                                                                                              LinearPD<double>,
                                                                                              PosePD>>>;


    /** Base acceleration index */
    iDynTree::IndexRange m_baseAccelerationIndex{iDynTree::IndexRange::InvalidRange()};
    /** Joint acceleration index */
    iDynTree::IndexRange m_jointAccelerationIndex{iDynTree::IndexRange::InvalidRange()};
    /** Control type index */
    iDynTree::IndexRange m_typeIndex{iDynTree::IndexRange::InvalidRange()};

    std::unique_ptr<ControllerType> m_controller; /**< Controller. */

    iDynTree::FrameIndex m_frameIndex; /**< Index of the frame. */

    iDynTree::MatrixDynSize m_jacobian; /**< Robotic jacobian of the element. */

    bool m_isInContact; /**< True if the frame if the link associated to the frame is in contact
                           with the environment. */
public:
    /* /\** type of the controller used in the CartesianElement *\/ */
    /* using ; */

    /**
     * Constructor
     * @param kinDyn KinDynComputations object
     * @param handler variable handler object
     * @param frameName name of the frame that has to be controlled (if CoM the CoM will be
     * controlled)
     * @throw std::runtime_error in case of an undefined variable/frame
     */
    CartesianElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                     std::unique_ptr<ControllerType> controller,
                     const VariableHandler& handler,
                     const std::string& frameName);

    /**
     * Set the desired trajectory
     * @param acceleration  desired angular acceleration
     * @param velocity desired  angular velocity
     * @param rotation desired rotation matrix
     */
    template <typename T, typename U, typename W>
    void setReference(const T& feedforward,
                      const U& referenceDerivative,
                      const W& reference);

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

    /**
     * Set if the link is in contact with the environment
     * @param isInContact true if the link associated to the frame is in contact with the
     * environment
     */
    void isInContact(bool isInContact);
};

} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers

#include "CartesianElements.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CONTROL_PROBLEM_ELEMENTS_H
