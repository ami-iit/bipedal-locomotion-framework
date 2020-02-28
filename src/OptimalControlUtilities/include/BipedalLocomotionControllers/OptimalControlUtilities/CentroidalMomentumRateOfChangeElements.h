/**
 * @file CentroidalMomentumRateOfChangeElements.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CENTROIDAL_MOMENTUM_RATE_OF_CHANGE_ELEMENT
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CENTROIDAL_MOMENTUM_RATE_OF_CHANGE_ELEMENT

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

    /** Vectors containing the frames in contact with the environment */
    std::unordered_map<std::string, FrameInContact<iDynTree::IndexRange, iDynTree::FrameIndex>> m_framesInContact;

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

    bool setMeasuredContactWrenches(const std::unordered_map<std::string, iDynTree::Wrench>& contactWrenches);

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


/**
 *
 */
class CentroidalAngularMomentumRateOfChangeElement : public ControlTask
{
    PIDController<iDynTree::Vector3> m_pid; /**< Linear PID */

    /* /\** Vectors containing the frames in contact with the environment *\/ */
    /* std::vector<Frame<iDynTree::IndexRange, iDynTree::FrameIndex>> m_framesInContact; */

    iDynTree::Vector3 m_zero; /**< Vector of zero elements */

    std::unordered_map<std::string, FrameInContactWithWrench<iDynTree::IndexRange, iDynTree::FrameIndex>> m_framesInContact;

    using FramesInContact = std::vector<FrameInContact<std::string, std::string>>;

    std::unique_ptr<Simulator::Integrator<iDynTree::Vector3>> m_angularMomentumIntegrator;
    std::unique_ptr<Simulator::Integrator<iDynTree::Vector3>> m_desiredAngularMomentumIntegrator;

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact.
     * @throw std::runtime_error if the frame is not defined
     */
    CentroidalAngularMomentumRateOfChangeElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                                 PIDController<iDynTree::Vector3> controller,
                                                 const VariableHandler& handler,
                                                 const FramesInContact& framesInContact,
                                                 const double& dT);

    /**
     */
    void setReference(const iDynTree::Vector3& centroidalAngularMomentumSecondDerivative,
                      const iDynTree::Vector3& centroidalAngularMomentumDerivative,
                      const iDynTree::Vector3& centroidalAngularMomentum);

    /**
     */
    void setGains(const iDynTree::Vector3& kd,  const iDynTree::Vector3& kp, const iDynTree::Vector3& ki);

    bool setMeasuredContactWrenches(
        const std::unordered_map<std::string, iDynTree::Wrench>& contactWrenches);

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

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CENTROIDAL_MOMENTUM_RATE_OF_CHANGE_ELEMENT
