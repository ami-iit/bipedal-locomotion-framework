/**
 * @file PDController.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_PD_CONTROLLER_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_PD_CONTROLLER_H

// iDynTree
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/Transform.h>

namespace BipedalLocomotionControllers
{
namespace OptimalControlUtilities
{
/**
 * PDController class is a virtual class that represents a generic PD controller.
 */
template <class T, class U, class W> class PDController
{
protected:
    T m_feedforward; /**< Desired feedforward. */
    U m_referenceDerivative; /**< Reference signal derivative. */
    W m_reference; /**< Reference signal. */

    U m_stateDerivative; /**< Derivative of the state. */
    W m_state; /**< State. */

    T m_controllerOutput; /**< Controller output. */

    bool m_controllerOutputEvaluated{false}; /**< Is the controller output evaluated? */

    /**
     * Evaluate the control output.
     */
    virtual void evaluateControl() = 0;

public:
    /**
     * Get the controller output.
     * @return controller output.
     */
    const T& getControllerOutput();

    /**
     * Set the desired trajectory.
     * @param feedforward
     * @param referenceDerivative derivative of the reference
     * @param reference reference signal
     */
    void
    setDesiredTrajectory(const T& feedforward, const U& referenceDerivative, const W& reference);

    /**
     * Set feedback
     * @param stateDerivative derivative of the state
     * @param state state of the syste,
     */
    void setFeedback(const U& stateDerivative, const W& state);
};

/**
 * RotationalPID implements the Rotational PID. For further information please refers to
 * http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.62.8655&rep=rep1&type=pdf,
 * section 5.11.6, p.173
 */
class OrientationPD : public PDController<iDynTree::Vector3, iDynTree::Vector3, iDynTree::Rotation>
{
    double m_c0; /**< Rotational PD Gain. */
    double m_c1; /**< Rotational PD Gain. */
    double m_c2; /**< Rotational PD Gain. */

    /**
     * Evaluate control
     */
    void evaluateControl() final;

public:

    /**
     * Transform a 3x3 matrix into a skew-symmetric matrix.
     * @param input is a 3x3 matrix;
     * @return a 3x3 skew-symmetric matrix
     */
    static iDynTree::Matrix3x3 skewSymmetric(const iDynTree::Matrix3x3& input);


    /**
     * Set rotational PID Gains
     * @param c0 pid gain;
     * @param c1 pid gain;
     * @param c2 pid gain.
     */
    void setGains(const double& c0, const double& c1, const double& c2);
};

/**
 * Standard Liner position PID
 */
template <class T> class LinearPD : public PDController<T, T, T>
{
    T m_kp; /**< Proportional gain */
    T m_kd; /**< Derivative gain */

    /**
     * Evaluate control
     */
    virtual void evaluateControl() final;

public:

    LinearPD(const T& kp, const T& kd);

    LinearPD() = default;

    /**
     * Set PID Gains
     * @param kp proportional gain (vector);
     * @param kd derivative gain (vector).
     */
    void setGains(const T& kp, const T& kd);
};

template <> void LinearPD<double>::evaluateControl();

template <> void LinearPD<iDynTree::VectorDynSize>::evaluateControl();

class PosePD : public PDController<iDynTree::SpatialAcc, iDynTree::Twist, iDynTree::Transform>
{
    double m_c0; /**< Rotational PD Gain. */
    double m_c1; /**< Rotational PD Gain. */
    double m_c2; /**< Rotational PD Gain. */

    iDynTree::Vector3 m_kp; /**< Proportional gain */
    iDynTree::Vector3 m_kd; /**< Derivative gain */

    /**
     * Evaluate control
     */
    void evaluateControl() final;

public:
    /**
     * Set PID Gains
     * @param kp proportional gain (vector);
     * @param kd derivative gain (vector).
     * @param c0 pid gain;
     * @param c1 pid gain;
     * @param c2 pid gain.
     */
    void setGains(const iDynTree::Vector3& kp, const iDynTree::Vector3& kd,
                  const double& c0, const double& c1, const double& c2);
};

} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers

#include "PDController.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_PD_CONTROLLER_H
