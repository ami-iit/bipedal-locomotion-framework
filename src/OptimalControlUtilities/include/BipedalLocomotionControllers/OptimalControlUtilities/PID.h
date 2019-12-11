/**
 * @file PID.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_PID_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_PID_H

// iDynTree
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/VectorFixSize.h>

namespace BipedalLocomotionControllers
{
/**
 * CartesianPID class is a virtual class that represents a generic Cartesian PID controller.
 */
class CartesianPID
{
protected:
    iDynTree::Vector3 m_desiredAcceleration; /**< Desired acceleration (feedforward). */
    iDynTree::Vector3 m_desiredVelocity; /**< Desired velocity. */

    iDynTree::Vector3 m_velocity; /**< Actual velocity. */

    iDynTree::Vector3 m_controllerOutput; /**< Controller output. */

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
    const iDynTree::Vector3& getControllerOutput();
};

/**
 * RotationalPID implements the Rotational PID. For further information please refers to
 * http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.62.8655&rep=rep1&type=pdf,
 * section 5.11.6, p.173
 */
class OrientationPID : public CartesianPID
{
    double m_c0; /**< Rotational PID Gain. */
    double m_c1; /**< Rotational PID Gain. */
    double m_c2; /**< Rotational PID Gain. */

    iDynTree::Rotation m_desiredOrientation; /**< Desired orientation. */
    iDynTree::Rotation m_orientation; /**< Actual orientation. */

    /**
     * Transform a 3x3 matrix into a skew-symmetric matrix.
     * @param input is a 3x3 matrix;
     * @return a 3x3 skew-symmetric matrix
     */
    iDynTree::Matrix3x3 skewSymmetric(const iDynTree::Matrix3x3& input);

    /**
     * Evaluate control
     */
    void evaluateControl() final;

public:
    /**
     * Set rotational PID Gains
     * @param c0 pid gain;
     * @param c1 pid gain;
     * @param c2 pid gain.
     */
    void setGains(const double& c0, const double& c1, const double& c2);

    /**
     * Set the desired trajectory.
     * @param desiredAcceleration desired acceleration (rad/s^2);
     * @param desiredVelocity desired velocity (rad/s);
     * @param desiredOrientation rotation matrix
     */
    void setDesiredTrajectory(const iDynTree::Vector3& desiredAcceleration,
                              const iDynTree::Vector3& desiredVelocity,
                              const iDynTree::Rotation& desiredOrientation);

    /**
     * Set feedback
     * @param velocity angular velocity:
     * @param orientation rotation matrix.
     */
    void setFeedback(const iDynTree::Vector3& velocity, const iDynTree::Rotation& orientation);
};

/**
 * Standard Liner position PID
 */
class PositionPID : public CartesianPID
{
    iDynTree::Vector3 m_kp; /**< Proportional gain */
    iDynTree::Vector3 m_kd; /**< Derivative gain */

    iDynTree::Vector3 m_desiredPosition; /**< Desired position. */

    iDynTree::Vector3 m_position; /**< Actual position. */

    iDynTree::Vector3 m_error; /**< Position error */
    iDynTree::Vector3 m_dotError; /**< Velocity error */

    /**
     * Evaluate control
     */
    void evaluateControl() final;

public:
    /**
     * Set PID Gains
     * @param kp proportional gain (scalar);
     * @param kd derivative gain (scalar).
     */
    void setGains(const double& kp, const double& kd);

    /**
     * Set PID Gains
     * @param kp proportional gain (vector);
     * @param kd derivative gain (vector).
     */
    void setGains(const iDynTree::Vector3& kp, const iDynTree::Vector3& kd);

    /**
     * Set the desired trajectory.
     * @param desiredAcceleration desired acceleration (m/s^2);
     * @param desiredVelocity desired velocity (m/s);
     * @param desiredPosition desired position (m).
     */
    void setDesiredTrajectory(const iDynTree::Vector3& desiredAcceleration,
                              const iDynTree::Vector3& desiredVelocity,
                              const iDynTree::Vector3& desiredPosition);

    /**
     * Set feedback
     * @param velocity linear velocity:
     * @param position actual position.
     */
    void setFeedback(const iDynTree::Vector3& velocity, const iDynTree::Vector3& position);
};

class OneDegreePID
{
    double m_desiredAcceleration; /**< Desired acceleration (feedforward). */
    double m_desiredVelocity; /**< Desired velocity. */
    double m_desiredPosition; /**< Desired position. */

    double m_velocity; /**< Actual velocity. */
    double m_position; /**< Actual position. */

    double m_controllerOutput; /**< Controller output. */

    double m_kp; /**< Proportional gain */
    double m_kd; /**< Derivative gain */

    bool m_controllerOutputEvaluated{false}; /**< Is the controller output evaluated? */

    /**
     * Evaluate the control output.
     */
    void evaluateControl();

public:
    /**
     * Set the gain pf the controller
     */
    void setGains(const double& kp, const double& kd);

    /**
     * Set the desired trajectory.
     * @param desiredAcceleration desired acceleration (m/s^2);
     * @param desiredVelocity desired velocity (m/s);
     * @param desiredPosition desired position (m).
     */
    void setDesiredTrajectory(const double& desiredAcceleration,
                              const double& desiredVelocity,
                              const double& desiredPosition);

    /**
     * Set feedback
     * @param velocity linear velocity:
     * @param position actual position.
     */
    void setFeedback(const double& velocity, const double& position);

    /**
     * Get the controller output.
     * @return controller output.
     */
    const double& getControllerOutput();
};
} // namespace BipedalLocomotionControllers
#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_PID_H
