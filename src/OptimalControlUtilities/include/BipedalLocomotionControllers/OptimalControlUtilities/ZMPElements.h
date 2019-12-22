/**
 * @file ZMPElement.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_ZMP_ELEMENT_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_ZMP_ELEMENT_H

#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>

namespace BipedalLocomotionControllers
{

namespace OptimalControlUtilities
{

/**
 * ZMPElement handles the tracking of the global Zero Moment Point (ZMP)
 */
class ZMPElement : public ControlTask
{
    std::vector<Frame> m_framesInContact; /**< Vectors containing the frames in contact with the
                                             environment */
    iDynTree::Vector2 m_ZMP; /**< Desired global Zero Moment Point position */
    iDynTree::Position m_contactFramePosition; /**< Position of the contact frame */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact. Each element of the vector is
     * a pair containing the name of the frame in the variableHandler and in the model.
     * @throw std::runtime_error if the frame is not defined
     */
    ZMPElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
               const VariableHandler& handler,
               const std::vector<std::pair<std::string, std::string>>& framesInContact);

    /**
     * Set the desired ZMP
     * @param ZMP position of the zero moment point
     */
    void setDesiredZMP(const iDynTree::Vector2& ZMP);

    /**
     * Get (and compute) the element matrix
     * @return the element matrix
     */
    virtual const iDynTree::MatrixDynSize& getA() final;
};

} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_ZMP_ELEMENT_H
