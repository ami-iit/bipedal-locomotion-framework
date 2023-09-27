/**
 * @file DistanceTask.h
 * @authors Ehsan Ranjbari
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef DISTANCE_TASK_H
#define DISTANCE_TASK_H

#include <memory>

#include <iDynTree/KinDynComputations.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>

namespace BipedalLocomotion
{
namespace IK
{

// clang-format off
/**
 * The DistanceTask class controls the distance of a frame with respect the world origin, or another frame.
 * The task assumes perfect control of the robot velocity \f$\nu\f$ that contains the base
 * linear and angular velocity expressed in mixed representation and the joint velocities.
 * The task represents the following equation
 * \f[
 * \frac{1}{d} p ^ \top J_p \nu = k (d ^ * - d)
 * \f]
 * where \f$p\f$ is the position of the target frame wrt the base or world frame.
 * \f$d\f$ is the current distance.
 * \f$J_p\f$ is the linear part of the robot jacobian.
 * \f$k\f$ is the controller gain and \f$d ^ *\f$ is the desired distance.
 */
// clang-format on
class DistanceTask : public IKLinearTask
{

    System::VariablesHandler::VariableDescription m_robotVelocityVariable; /**< Variable describing
                                                                              the robot velocity
                                                                              (base + joint) */

    static constexpr std::size_t m_spatialVelocitySize{6}; /**< Size of the spatial velocity vector.
                                                            */
    static constexpr std::size_t m_DoFs{1}; /**< DoFs associated to the task */

    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

    Eigen::MatrixXd m_jacobian, m_relativeJacobian; /**< Internal buffers to store the jacobian. */
    Eigen::Vector3d m_framePosition; /**< Internal buffer to store the frame position. */
    double m_kp; /**< Controller gain. */
    double m_desiredDistance{0.0}; /**< Desired distance. */

    std::string m_referenceName; /**< Reference frame name. */
    std::string m_targetFrameName; /**< Target frame name. */
    iDynTree::FrameIndex m_referenceFrameIndex, m_targetFrameIndex; /**< Base and target frame name
                                                                       indexes. */

    double m_computedDistance{0.0}; /**< Computed distance. */
    double m_distanceNumericThreshold{0.001}; /**< Numeric threshold when inverting the computed
                                                 distance in the Jacobian computation. */

public:
    // clang-format off
    /**
     * Initialize the task.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |              Type            |                                                        Description                                                                       | Mandatory |
     * |:----------------------------------:|:----------------------------:|:----------------------------------------------------------------------------------------------------------------------------------------:|:---------:|
     * |   `robot_velocity_variable_name`   |            `string`          |                            Name of the variable contained in `VariablesHandler` describing the robot velocity                            |    Yes    |
     * |                `kp`                |            `double`          |                                               Gain of the distance controller                                                            |    Yes    |
     * |         `target_frame_name`        |            `string`          |                                       Name of the frame of which computing the distance                                                  |    Yes    |
     * |       `reference_frame_name`       |            `string`          |          Name of the frame with respect to which computing the distance. If empty, the world frame will be used (Default = "")           |    No     |
     * |    `distance_numeric_threshold`    |            `double`          |                     Lowerbound for the computed distance when inverting it in the task Jacobian (Default = 0.001)                        |    No     |
     * @return True in case of success, false otherwise.
     */
    // clang-format on
    bool
    initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;

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
     * Update the content of the task.
     * @return True in case of success, false otherwise.
     */
    bool update() override;

    /**
     * @brief setDesiredDistance Set the desired distance between the target and the reference frame
     * @param desiredDistance The desired distance expressed in meters
     * @note The desired distance is considered without sign.
     * @return True in case of success, false otherwise.
     */
    bool setDesiredDistance(double desiredDistance);

    /**
     * @brief setSetPoint Set the desired distance between the target and the reference frame
     * @param desiredDistance The desired distance expressed in meters
     *
     * It is equivalent to setDesiredDistance
     *
     * @note The desired distance is considered without sign.
     * @return True in case of success, false otherwise.
     */
    bool setSetPoint(double desiredDistance);

    /**
     * @brief Get the computed distance between the target frame and either the world origin, or the
     * base origin.
     * @return The computed distance
     */
    double getDistance() const;

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;

    /**
     * The DistanceTask is an equality task.
     * @return the size of the task.
     */
    Type type() const override;

    /**
     * Determines the validity of the objects retrieved with getA() and getB()
     * @return True if the objects are valid, false otherwise.
     */
    bool isValid() const override;
};

BLF_REGISTER_IK_TASK(DistanceTask);

} // namespace IK
} // namespace BipedalLocomotion

#endif // DISTANCE_TASK_H
