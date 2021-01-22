/**
 * @file JointsTrackingTask.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_TSID_JOINT_REGULARIZATION_TASK_H
#define BIPEDAL_LOCOMOTION_TSID_JOINT_REGULARIZATION_TASK_H

#include <BipedalLocomotion/TSID/Task.h>

#include <LieGroupControllers/ProportionalDerivativeController.h>

namespace BipedalLocomotion
{
namespace TSID
{

/**
 * JointsTrackingTask is a concrete implementation of the Task. Please use this element if you
 * want to control the desired joint position of the robot.
 * The task represents the following equation
 * \f[
 * \begin{bmatrix} 0_6 & I_n \end{bmatrix} \nu = \ddot{s} ^ *
 * \f]
 * where \f$0_6\f$ and \f$I_n\f$ are the zero and the identity matrix.
 * The desired joint acceleration is chosen such that the joint will converge to the desired
 * trajectory and it is computed with a standard standard PD controller in \f$\mathbb{R}^n\f$.
 */
class JointsTrackingTask : public Task
{
    Eigen::VectorXd m_kp; /**< Proportional gain. */
    Eigen::VectorXd m_kd; /**< Derivative gain. */
    Eigen::VectorXd m_jointVelocity; /**< Joints velocity in radians per second. */
    Eigen::VectorXd m_jointPosition; /**< Joints position in radians */
    Eigen::VectorXd m_desiredJointVelocity; /**< Desired joints velocity in radians per second. */
    Eigen::VectorXd m_desiredJointAcceleration; /**< Desired joints acceleration in radians per
                                                   second square. */
    Eigen::VectorXd m_desiredJointPosition; /**< Desired joints position in radians. */
    Eigen::VectorXd m_zero; /**< Vector containing zero elements. */

public:
    /**
     * Initialize the planner.
     * @param paramHandler pointer to the parameters handler.
     * @param variablesHandler reference to a variables handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                      | Mandatory |
     * |:----------------------------------:|:--------:|:--------------------------------------------------------------------------------------:|:---------:|
     * | `robot_acceleration_variable_name` | `string` | Name of the variable contained in `VariablesHandler` describing the robot acceleration |    Yes    |
     * |             `kp       `            | `vector` |                                Proportional controller gain                            |    Yes    |
     * |             `kd`                   | `vector` |      Derivative Gain of the controller. if not specified \f$k_d = 2 \sqrt{k_p}\f$      |    No     |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> paramHandler,
                    const System::VariablesHandler& variablesHandler) override;

    /**
     * Update the content of the element.
     * @return True in case of success, false otherwise.
     */
    bool update() override;

    /**
     * Set the desired reference trajectory.
     * @param jointPosition vector containing the desired joint position in radians.
     * @note The desired velocity and acceleration are implicitly set to zero.
     * @return True in case of success, false otherwise.
     */
    bool setReferenceTrajectory(Eigen::Ref<const Eigen::VectorXd> jointPosition);

    /**
     * Set the desired reference trajectory.
     * @param jointPosition vector containing the desired joint position in radians.
     * @param jointVelocity vector containing the desired joint velocity in radians per second.
     * @note The desired acceleration is implicitly set to zero.
     * @return True in case of success, false otherwise.
     */
    bool setReferenceTrajectory(Eigen::Ref<const Eigen::VectorXd> jointPosition,
                                Eigen::Ref<const Eigen::VectorXd> jointVelocity);

    /**
     * Set the desired reference trajectory.
     * @param jointPosition vector containing the desired joint position in radians.
     * @param jointVelocity vector containing the desired joint velocity in radians per second.
     * @param jointAcceleration vector containing the desired joint velocity in radians per second square.
     * @return True in case of success, false otherwise.
     */
    bool setReferenceTrajectory(Eigen::Ref<const Eigen::VectorXd> jointPosition,
                                Eigen::Ref<const Eigen::VectorXd> jointVelocity,
                                Eigen::Ref<const Eigen::VectorXd> jointAcceleration);
};

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_JOINT_REGULARIZATION_TASK_H
