/**
 * @file JointTrackingTask.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_JOINT_TRACKING_TASK_H
#define BIPEDAL_LOCOMOTION_IK_JOINT_TRACKING_TASK_H

#include <memory>

#include <iDynTree/KinDynComputations.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>

#include <LieGroupControllers/ProportionalDerivativeController.h>

namespace BipedalLocomotion
{
namespace IK
{

/**
 * JointsTrackingTask is a concrete implementation of the Task. Please use this element if you
 * want to control the desired joint position of the robot.
 * The task represents the following equation
 * \f[
 * \begin{bmatrix} 0_6 & I_n \end{bmatrix} \nu = \dot{s} ^ *
 * \f]
 * where \f$0_6\f$ and \f$I_n\f$ are the zero and the identity matrix.
 * The desired joint velocity is chosen such that the joint will converge to the desired
 * trajectory and it is computed with a standard standard PD controller in \f$\mathbb{R}^n\f$.
 */
class JointTrackingTask : public IKLinearTask
{
    Eigen::VectorXd m_kp; /**< Proportional gain. */
    Eigen::VectorXd m_jointPosition; /**< Joints position in radians */
    Eigen::VectorXd m_desiredJointVelocity; /**< Desired joint velocities in radians per second. */
    Eigen::VectorXd m_desiredJointPosition; /**< Desired joints position in radians. */
    Eigen::VectorXd m_zero; /**< Vector containing zero elements. */

    std::string m_robotVelocityVariableName;

    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

    bool m_isSetPointSetAtLeastOnce{false}; /**< True if the set point has been set at least once. */                                                               
public:
    /**
     * Initialize the task.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                      | Mandatory |
     * |:----------------------------------:|:--------:|:--------------------------------------------------------------------------------------:|:---------:|
     * |    `robot_velocity_variable_name`  | `string` |   Name of the variable contained in `VariablesHandler` describing the robot velocity   |    Yes    |
     * |             `kp       `            | `vector` |                                Proportional controller gain                            |    Yes    |
     * @return True in case of success, false otherwise.
     * Where the generalized robot velocity is a vector containing the base spatial-velocity
     * (expressed in mixed representation) and the joint velocities.
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;

    /**
     * Set the kinDynComputations object.
     * @param kinDyn pointer to a kinDynComputations object.
     * @return True in case of success, false otherwise.
     */
    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn) override;

    /**
     * Set the set of variables required by the task. The variables are stored in the
     * System::VariablesHandler.
     * @param variablesHandler reference to a variables handler.
     * @note The handler must contain a variable named as the parameter
     * `robot_velocity_variable_name` stored in the parameter handler. The variable represents the
     * generalized velocity of the robot. Where the generalized robot velocity is a vector
     * containing the base spatial-velocity (expressed in mixed representation) and the joints
     * velocity.
     * @return True in case of success, false otherwise.
     */
    bool setVariablesHandler(const System::VariablesHandler& variablesHandler) override;

    /**
     * Update the content of the element.
     * @return True in case of success, false otherwise.
     */
    bool update() override;

    /**
     * Set the desired setpoint.
     * @param jointPosition vector containing the desired joint position in radians.
     * @note The desired velocity is implicitly set to zero.
     * @return True in case of success, false otherwise.
     */
    bool setSetPoint(Eigen::Ref<const Eigen::VectorXd> jointPosition);

    /**
     * Set the desired setpoint.
     * @param jointPosition vector containing the desired joint position in radians.
     * @param jointVelocity vector containing the desired joint velocity in radians per second.
     * @return True in case of success, false otherwise.
     */
    bool setSetPoint(Eigen::Ref<const Eigen::VectorXd> jointPosition,
                     Eigen::Ref<const Eigen::VectorXd> jointVelocity);

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;

    /**
     * The JointsTrackingTask is an equality task.
     * @return the size of the task.
     */
    Type type() const override;

    /**
     * Determines the validity of the objects retrieved with getA() and getB()
     * @return True if the objects are valid, false otherwise.
     */
    bool isValid() const override;
};

BLF_REGISTER_IK_TASK(JointTrackingTask);

} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_IK_JOINT_TRACKING_TASK_H
