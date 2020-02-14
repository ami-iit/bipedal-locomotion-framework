/**
 * @file PIDController.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_PID_CONTROLLER_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_PID_CONTROLLER_H

namespace BipedalLocomotionControllers
{
namespace OptimalControlUtilities
{
/**
 * PDController class is a virtual class that represents a generic PD controller.
 */
template <class T> class PIDController
{
protected:
    T m_feedforward; /**< Desired feedforward. */
    T m_referenceDerivative; /**< Reference signal derivative. */
    T m_reference; /**< Reference signal. */
    T m_referenceIntegral; /**< Reference signal. */

    T m_stateDerivative; /**< Derivative of the state. */
    T m_state; /**< State. */
    T m_stateIntegral; /**< State. */

    T m_controllerOutput; /**< Controller output. */

    T m_kp; /**< Proportional gain */
    T m_kd; /**< Derivative gain */
    T m_ki; /**< Integral gain */

    bool m_controllerOutputEvaluated{false}; /**< Is the controller output evaluated? */

    /**
     * Evaluate the control output.
     */
    void evaluateControl();

public:

    PIDController(const T& kd, const T& kp, const T& ki);

    /**
     * Get the controller output.
     * @return controller output.
     */
    const T& getControllerOutput();

    void setGains(const T& kd, const T& kp, const T& ki);


    /**
     * Set the desired trajectory.
     * @param feedforward
     * @param referenceDerivative derivative of the reference
     * @param reference reference signal
     */
    void setReference(const T& feedforward, const T& referenceDerivative, const T& reference, const T& referenceIntegral);

    /**
     * Set feedback
     * @param stateDerivative derivative of the state
     * @param state state of the syste,
     */
    void setFeedback(const T& stateDerivative, const T& state, const T& stateIntegral);
};
} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers

#include "PIDController.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_PD_CONTROLLER_H
