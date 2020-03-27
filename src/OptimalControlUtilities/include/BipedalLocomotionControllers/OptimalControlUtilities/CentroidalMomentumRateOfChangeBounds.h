/**
 * @file CentroidalMomentumRateOfChangeBounds.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CENTROIDAL_MOMENTUM_RATE_OF_CHANGE_BOUNDS
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CENTROIDAL_MOMENTUM_RATE_OF_CHANGE_BOUNDS

#include <memory>

#include <iDynTree/Core/VectorFixSize.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/PIDController.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/Frame.h>

#include <BipedalLocomotionControllers/Simulator/Integrator.h>

namespace BipedalLocomotionControllers
{

namespace OptimalControlUtilities
{

/**
 *
 */
class CentroidalAngularMomentumRateOfChangeBounds : public InequalityConstraintElement
{
    double m_dT;

    iDynTree::Vector3 m_angularMomentumLowerBound;
    iDynTree::Vector3 m_angularMomentumUpperBound;

    iDynTree::Vector3 m_zero; /**< Vector of zero elements */

    std::unordered_map<std::string, FrameInContactWithWrench<iDynTree::IndexRange, iDynTree::FrameIndex>> m_framesInContact;

    using FramesInContact = std::vector<FrameInContact<std::string, std::string>>;

    iDynTree::Vector3 m_angularMomentumDerivative;

    void computeBound(const iDynTree::Vector3& angularMomentumBound, iDynTree::VectorDynSize& bound);

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact.
     * @throw std::runtime_error if the frame is not defined
     */
    CentroidalAngularMomentumRateOfChangeBounds(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                                const VariableHandler& handler,
                                                const FramesInContact& framesInContact,
                                                const iDynTree::Vector3& angularMomentumUpperBound,
                                                const iDynTree::Vector3& angularMomentumLowerBound,
                                                const double& dT);

    bool setMeasuredContactWrenches(
        const std::unordered_map<std::string, iDynTree::Wrench>& contactWrenches);

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


} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CENTROIDAL_MOMENTUM_RATE_OF_CHANGE_BOUNDS
