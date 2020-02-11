/**
 * @file FeasibilityElements.h
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
