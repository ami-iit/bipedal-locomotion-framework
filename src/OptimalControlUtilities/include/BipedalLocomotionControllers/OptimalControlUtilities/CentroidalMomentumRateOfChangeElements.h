/**
 * @file CentroidalMomentumRateOfChangeElements.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CENTROIDAL_MOMENTUM_RATE_OF_CHANGE_ELEMENT
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CENTROIDAL_MOMENTUM_RATE_OF_CHANGE_ELEMENT

#include <iDynTree/Core/VectorFixSize.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/PIDController.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/Frame.h>

#include <BipedalLocomotionControllers/ContactModels/ContactModel.h>

namespace BipedalLocomotionControllers
{

namespace OptimalControlUtilities
{

/**
 * CentroidalLinearMomentumElementWithCompliantContact describes the Centroidal linear momentum
 * dynamics when the contact between the robot and the environment are modelled as compliant
 */
class CentroidalLinearMomentumRateOfChangeElement : public ControlTask
{
    PIDController<iDynTree::Vector3> m_pid; /**< Linear PID */

    iDynTree::LinearForceVector3 m_robotWeight; /**< Weight of the robot expressed in the inertial
                                                   frame (The z axis points upwards) */
    double m_robotMass;

    using FramesInContact = std::vector<FrameInContact<std::string, std::string>>;

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact. Each element of the
     * vector is the name of the frame in the variableHandler
     * @throw std::runtime_error if the frame is not defined
     */
    CentroidalLinearMomentumRateOfChangeElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                                PIDController<iDynTree::Vector3> controller,
                                                const VariableHandler& handler,
                                                const FramesInContact& framesInContact);

    /**
     * Set the desired centroidal linear momentum
     * @param centroidalLinearMomentumSecondDerivative desired second derivative of the centroidal linear momentum
     * @param centroidalLinearMomentumDerivative desired derivative of the centroidal linear momentum
     * @param centroidalLinearMomentum desired centroidal linear momentum
     */
    void setReference(const iDynTree::Vector3& centroidalLinearMomentumSecondDerivative,
                      const iDynTree::Vector3& centroidalLinearMomentumDerivative,
                      const iDynTree::Vector3& centroidalLinearMomentum,
                      const iDynTree::Vector3& centerOfMass) noexcept;

    void setMeasuredContactForces(const std::vector<iDynTree::LinearForceVector3>& contactForces);

    /**
     * Set the controller gains
     * @param kp proportional gain
     * @param kd derivative gain
     */
    void setGains(const iDynTree::Vector3& kd,  const iDynTree::Vector3& kp, const iDynTree::Vector3& ki);

    /**
     * Get (and compute) the element vector
     * @return the element vector
     */
    virtual const iDynTree::VectorDynSize& getB() final;
};

} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CENTROIDAL_MOMENTUM_RATE_OF_CHANGE_ELEMENT
