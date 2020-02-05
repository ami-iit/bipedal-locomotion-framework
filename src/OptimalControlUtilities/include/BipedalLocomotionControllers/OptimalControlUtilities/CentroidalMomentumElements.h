/**
 * @file CentroidalMomentumElements.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CENTROIDAL_MOMENTUM_ELEMENT_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CENTROIDAL_MOMENTUM_ELEMENT_H

#include <iDynTree/Core/VectorFixSize.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/PDController.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>

namespace BipedalLocomotionControllers
{

namespace OptimalControlUtilities
{

/**
 * CentroidalLinearMomentumElement describes the Centroidal linear momentum
 */
class CentroidalLinearMomentumElement : public ControlTask
{
    std::unique_ptr<LinearPD<iDynTree::Vector3>> m_pd; /**< Linear PD */
    iDynTree::Vector3 m_zero; /**< Vector of zero elements */
    double m_robotMass; /**< Mass of the robot */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact. Each element of the
     * vector is the name of the frame in the variableHandler
     * @throw std::runtime_error if the frame is not defined
     */
    CentroidalLinearMomentumElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                    const VariableHandler& handler,
                                    const std::vector<std::string>& framesInContact);

    /**
     * Set the desired centroidal linear momentum
     * @param centroidalLinearMomentumDerivative desired linear momentum rate of change
     * @param centroidalLinearMomentum desired linear momentum
     */
    void
    setDesiredCentroidalLinearMomentum(const iDynTree::Vector3& centroidalLinearMomentumDerivative,
                                       const iDynTree::Vector3& centroidalLinearMomentum);

    /**
     * Set the controller gain
     * @param kp controller gain
     */
    void setGain(const iDynTree::Vector3& kp);

    /**
     * Get (and compute) the element vector
     * @return the element vector
     */
    virtual const iDynTree::VectorDynSize& getB() final;
};

/**
 * CentroidalAngularMomentumElement describes the Centroidal angular momentum
 */
class CentroidalAngularMomentumElement : public ControlTask
{
private:
    std::unique_ptr<LinearPD<iDynTree::Vector3>> m_pd; /**< Linear PD */

    /** Vectors containing the frames in contact with the environment */
    std::vector<Frame<iDynTree::IndexRange, iDynTree::FrameIndex>> m_framesInContact;

    iDynTree::Vector3 m_zero; /**< Vector of zero elements */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact.
     * @throw std::runtime_error if the frame is not defined
     */
    CentroidalAngularMomentumElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                     const VariableHandler& handler,
                                     const std::vector<Frame<std::string, std::string>>& framesInContact);

    /**
     * Set the desired centroidal angular momentum
     * @param centroidalAngularMomentumDerivative desired angular momentum rate of change
     * @param centroidalAngularMomentum desired angular momentum
     */
    void
    setDesiredCentroidalAngularMomentum(const iDynTree::Vector3& centroidalAngularMomentumDerivative,
                                        const iDynTree::Vector3& centroidalAngularMomentum);

    /**
     * Set the controller gain
     * @param kp controller gain
     */
    void setGain(const iDynTree::Vector3& kp);

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

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CENTROIDAL_MOMENTUM_ELEMENT_H
