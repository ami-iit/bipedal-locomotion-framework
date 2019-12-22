/**
 * @file FeasibilityElement.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_FEASIBILITY_ELEMENT_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_FEASIBILITY_ELEMENT_H

#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>

namespace BipedalLocomotionControllers
{

namespace OptimalControlUtilities
{

/**
 * ContactWrenchFeasibilityElement handles the constraints related to the contact wrenches
 * such that: Unilateral constraint, friction cone, torsional friction parameter
 * and position of the local Center of Pressure
 */
class ContactWrenchFeasibilityElement : public InequalityConstraintElement
{
    Frame m_frameInContact; /**< Frame in contact with the environment */
    iDynTree::Rotation m_rotationMatrix; /**< Frame rotation matrix */
    iDynTree::MatrixDynSize m_AInBodyFrame; /**< Constrain matrix written in body frame */
    double m_infinity; /**< Double representing the infinity */
    double m_minimalNormalForce; /**< Minimal normal force required */
    unsigned int m_nominalForceConstraintIndex; /**< Index of the minimal normal force required */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param frameInContact pair containing the frames in contact. The first
     * element is the name of the frame in the handler while the second is
     * the name in the model
     * @param numberOfPoints number of points used to approximate a quarter of the friction cone
     * @param staticFrictionCoefficient static friction coefficient
     * @param torsionalFrictionCoefficient torsional friction coefficient
     * @param minimalNormalForce minimal normal force required
     * @param footLimitX foot size on the X axis written with respect the foot frame. The
     * first value should be negative while the second one positive
     * @param footLimitY foot size on the Y axis written with respect the foot frame. The
     * first value should be negative while the second one positive
     * @param infinity double representing the infinity
     * @throw std::runtime_error if the frame is not defined
     */
    ContactWrenchFeasibilityElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                    const VariableHandler& handler,
                                    const std::pair<std::string, std::string>& frameInContact,
                                    const int& numberOfPoints,
                                    const double& staticFrictionCoefficient,
                                    const double& torsionalFrictionCoefficient,
                                    const double& minimalNormalForce,
                                    const iDynTree::Vector2& footLimitX,
                                    const iDynTree::Vector2& footLimitY,
                                    const double& infinity);

    /**
     * Set if the link is in contact with the environment
     * @param isInContact true if the link associated to the frame is in contact with the
     * environment
     */
    void isInContact(bool isInContact);

    /**
     * Get (and compute) the element matrix
     * @return the element matrix
     */
    virtual const iDynTree::MatrixDynSize& getA() final;

    /**
     * Get the upper bound
     * @return the upper bound
     */
    virtual const iDynTree::VectorDynSize& getUpperBound() final;

    /**
     * Get the lower bound
     * @return the lower bound
     */
    virtual const iDynTree::VectorDynSize& getLowerBound() final;
};

/**
 * JointValuesFeasibilityElement handles the robot joint limits
 */
class JointValuesFeasibilityElement : public InequalityConstraintElement
{
    /** Index of the joint acceleration */
    iDynTree::IndexRange m_jointAccelerationIndex{iDynTree::IndexRange::InvalidRange()};
    iDynTree::VectorDynSize m_jointPositions; /**< Joint position in radiant */
    iDynTree::VectorDynSize m_jointVelocities; /**< Joint velocities in radiant per second */

    iDynTree::VectorDynSize m_minJointPositionsLimit; /**< Min joint position limit in radiant */
    iDynTree::VectorDynSize m_maxJointPositionsLimit; /**< Max joint position limit in radiant */

    double m_samplingTime; /**< Sampling time */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param variableName name of the variable in the handler
     * @param maxJointPositionsLimit Max joint position limit in radiant
     * @param minJointPositionsLimit Min joint position limit in radiant
     * @param samplingTime the controller rate in second
     * @throw std::runtime_error if variableName is not defined in the handler
     */
    JointValuesFeasibilityElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                  const VariableHandler& handler,
                                  const std::string& variableName,
                                  const iDynTree::VectorDynSize& maxJointPositionsLimit,
                                  const iDynTree::VectorDynSize& minJointPositionsLimit,
                                  const double& samplingTime);

    /**
     * Get the upper bound
     * @return the upper bound
     */
    virtual const iDynTree::VectorDynSize& getUpperBound() final;

    /**
     * Get the lower bound
     * @return the lower bound
     */
    virtual const iDynTree::VectorDynSize& getLowerBound() final;
};

} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_FEASIBILITY_ELEMENT_H
