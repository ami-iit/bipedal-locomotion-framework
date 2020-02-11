/**
 * @file ContactModelElement.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CONTACT_MODEL_ELEMENT_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CONTACT_MODEL_ELEMENT_H

#include <iDynTree/Core/VectorFixSize.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/PDController.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/Frame.h>

#include <BipedalLocomotionControllers/ContactModels/ContactModel.h>

namespace BipedalLocomotionControllers
{

namespace OptimalControlUtilities
{

/**
 * ContactModelElement
 */
class ContactModelElement : public ControlTask
{
    /** Frame in contact with the environment */
    FrameInContactWithContactModel<iDynTree::IndexRange, iDynTree::FrameIndex> m_frameInContact;

    iDynTree::MatrixDynSize m_jacobianMatrix; /**< Jacobian Matrix  */

    /** Base acceleration index */
    iDynTree::IndexRange m_baseAccelerationIndex{iDynTree::IndexRange::InvalidRange()};
    /** Joint acceleration index */
    iDynTree::IndexRange m_jointAccelerationIndex{iDynTree::IndexRange::InvalidRange()};
public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact. Each element of the
     * vector is the name of the frame in the variableHandler
     * @throw std::runtime_error if the frame is not defined
     */
    ContactModelElement(
        std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
        const VariableHandler& handler,
        const FrameInContactWithContactModel<std::string, std::string>& frameInContact);

    void setContactState(bool isInContact,
                         const iDynTree::Transform& nullForceTransform);

    /**
     * Get (and compute) the element vector
     * @return the element vector
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

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CONTACT_MODEL_ELEMENT_H
