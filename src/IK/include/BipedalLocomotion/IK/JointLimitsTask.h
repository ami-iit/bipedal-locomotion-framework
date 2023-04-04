/**
 * @file JointLimitsTask.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_JOINT_LIMITS_TASK_H
#define BIPEDAL_LOCOMOTION_IK_JOINT_LIMITS_TASK_H

#include <chrono>
#include <memory>
#include <vector>

#include <iDynTree/KinDynComputations.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>

namespace BipedalLocomotion
{
namespace IK
{

/**
 * JointLimitsTask is a concrete implementation of the Task. Please use this element if you
 * want to ensure that the generated joint velocity will consider the joints position limits.
 * The task represents the following equation
 * \f[
 * \dot{s}_{-} \preceq \begin{bmatrix} 0_6 & I_n \end{bmatrix} \nu \preceq \dot{s}_{+}
 * \f]
 * where \f$0_6\f$ and \f$I_n\f$ are the zero and the identity matrix. \f$\dot{s} _{-}\f$ and
 * \f$\dot{s} _{+}\f$ are chosen such that robot stays within its mechanical range of motion.
 * Following [(Kanoun, 2011)](https://ieeexplore.ieee.org/document/6301071), we set
 * \f$\dot{s}_{-}\f$ and \f$\dot{s}_{+}\f$ as follows
 * \f[
 * \dot{s}_{-} = k_{\text{lim}} \frac{s_{-} - s}{dt} \quad \dot{s}_{+} = k_{\text{lim}} \frac{s_{+} - s}{dt}
 * \f]
 * where \f${s}_{-}\f$ and \f${s} _{+}\f$ are the joints mechanical limits, and \f$k_{\text{lim}}\f$
 * is a proportional postive gain lower than 1. \f$k_{\text{lim}}\f$, for instance, denotes that a
 * joint angle update  will not increase by more than half the distance between the current joint
 * value and its boundaries.
 */
class JointLimitsTask : public IKLinearTask
{
    Eigen::VectorXd m_klim; /**< Proportional gain. */
    Eigen::VectorXd m_jointPosition; /**< Joints position in radians */
    Eigen::VectorXd m_upperLimits; /**< Upper limits in radians. */
    Eigen::VectorXd m_lowerLimits; /**< Lower limits in radians. */

    bool m_isLimitConsideredForAllJoints{false};

    std::chrono::nanoseconds m_samplingTime;

    std::string m_robotVelocityVariableName;

    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

public:
    /**
     * Initialize the task.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                            Description                                                      | Mandatory |
     * |:----------------------------------:|:--------:|:-----------------------------------------------------------------------------------------------------------:|:---------:|
     * |           `sampling_time`          | `double` |                                             Sampling time in seconds                                        |    Yes    |
     * |    `robot_velocity_variable_name`  | `string` |             Name of the variable contained in `VariablesHandler` describing the robot velocity              |    Yes    |
     * |             `klim`                 | `vector` |                     Proportional controller gain. It must be a positive number lower than 1                 |    Yes    |
     * |        `use_model_limits`          |  `bool`  |           If true the limits will be taken from the urdf otherwise the user should provide them             |    Yes    |
     * |          `upper_limits`            | `vector` | Vector containing \f$s_{+}\f$ such that \f$ s \le s_{+} \f$. Required `use_model_limits` is set to false.   |    No     |
     * |          `lower_limits`            | `vector` | Vector containing \f$s_{-}\f$ such that \f$ s \ge s_{-} \f$. Required `use_model_limits` is set to false.   |    No     |
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
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;

    /**
     * The JointsLimitsTask is an inequality task.
     * @return the size of the task.
     */
    Type type() const override;

    /**
     * Determines the validity of the objects retrieved with getA() and getB()
     * @return True if the objects are valid, false otherwise.
     */
    bool isValid() const override;
};

BLF_REGISTER_IK_TASK(JointLimitsTask);

} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_IK_JOINT_LIMITS_TASK_H
