/**
 * @file DistanceTask.h
 * @authors Ehsan Ranjbari
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef GRAVITY_TASK_H
#define GRAVITY_TASK_H

#include <memory>

#include <iDynTree/KinDynComputations.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>

#include <LieGroupControllers/ProportionalDerivativeController.h>

namespace BipedalLocomotion
{
namespace IK
{

// clang-format off
/**
 * The GravityTask class aligns the z axis of a frame to a desired gravity direction.
 * The task assumes perfect control of the robot velocity \f$\nu\f$ that contains the base
 * linear and angular velocity expressed in mixed representation and the joint velocities.
 * The task represents the following equation
 * \f[
 * \begin{bmatrix}
 * 1 & 0 & 0 \\
 * 0 & 1 & 0
 * \end{bmatrix} J_\omega \nu = k \begin{bmatrix}
 * 0 &-1 & 0 \\
 * 1 & 0 & 0
 * \end{bmatrix} {} ^ W R_f {} ^ f g ^ * + {} ^ W R_f {} ^ f \omega_{ff}
 * \f]
 * where \f$J_\omega\f$ is the angular part of the frame jacobian.
 * \f$k\f$ is the controller gain.
 * \f${} ^ W R_f\f$ is the frame rotation.
 * \f${} ^ W g ^ *\f$ is the desired z axis direction expressed in the frame coordinates.
 * \f${} ^ W \omega_{ff}\f$ is the feed forward velocity expressed in the frame coordinates.
 * A more through explanation is provided in https://github.com/ami-iit/bipedal-locomotion-framework/issues/651
 */
// clang-format on
class GravityTask : public IKLinearTask
{

    System::VariablesHandler::VariableDescription m_robotVelocityVariable; /**< Variable describing
                                                                              the robot velocity
                                                                              (base + joint) */

    static constexpr std::size_t m_spatialVelocitySize{6}; /**< Size of the spatial velocity vector.
                                                            */
    static constexpr std::size_t m_DoFs{2}; /**< DoFs associated to the task */

    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

    double m_kp; /**< Controller gain. */
    Eigen::MatrixXd m_jacobian; /**< Internal buffer to store the jacobian. */
    Eigen::Vector3d m_desiredZDirectionBody; /**< Desired gravity direction in the target frame. */
    Eigen::Vector3d m_feedForwardBody; /**< Feedforward term expressed in body direction. */
    Eigen::MatrixXd m_Am; /**< Matrix filtering jacobian rows. */
    Eigen::MatrixXd m_bm; /**< Matrix filtering acceleration. */

    iDynTree::FrameIndex m_targetFrameIndex; /**< Index of the target frame. */

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
     * |         `target_frame_name`        |            `string`          |                                       Name of the frame to which apply the gravity task                                                  |    Yes    |
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
     * @brief Set the desired gravity direction expressed the target frame.
     * @param desiredGravityDirection The desired gravity direction.
     *
     * The input is normalized, unless the norm is too small.
     */
    void setDesiredGravityDirectionInTargetFrame(
        const Eigen::Ref<const Eigen::Vector3d> desiredGravityDirection);

    /**
     * @brief Set the feedforward angular velocity expressed in the target frame.
     * @param feedforwardVelocity The desired feedforward velocity
     *
     * Only the first two components are used.
     */
    void
    setFeedForwardVelocityInTargetFrame(const Eigen::Ref<const Eigen::Vector3d> feedforwardVelocity);

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

BLF_REGISTER_IK_TASK(GravityTask);

} // namespace IK
} // namespace BipedalLocomotion

#endif // GRAVITY_TASK_H
