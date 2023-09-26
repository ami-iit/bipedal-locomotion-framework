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
 * \end{bmatrix} J_\omega \nu = -k \begin{bmatrix}
 * 0 &-1 & 0 \\
 * 1 & 0 & 0
 * \end{bmatrix} {} ^ W R_f {} ^ f g ^ * + {} ^ W R_f {} ^ f \omega_{ff}
 * \f]
 * where \f$J_\omega\f$ is the angular part of the frame jacobian.
 * \f$k\f$ is the controller gain.
 * \f${} ^ W R_f\f$ is the frame rotation.
 * \f${} ^ W g ^ *\f$ is the desired z axis direction expressed in the frame coordinates.
 * \f${} ^ W \omega_{ff}\f$ is the feed-forward velocity expressed in the frame coordinates.
 * The explanation is as follows.
 *
 * We define ${}^wR_b$ as the orientation of the frame $b$ we want to move,
 * and ${}^b\hat{V}$ as the unitary vector fixed in frame $b$ that we want to align to ${}^wz = e_3 = [0 0 1]^\top$.
 *
 * We define the following Lyapunov function:
 * \f[
 * V = 1 - \cos(\theta) \geq 0.
 * \f]
 *
 * Since \f$ a^\top b = ||a|| ~||b|| \cos(\theta) \f$, we have:
 * \f[
 * V = 1 - \left({}^wR_b{}^b\hat{V}\right)^\top e_3,
 * \f]
 * given that both vectors are unitary.
 *
 * The time derivative of the Lyapunov function is as follows:
 * \f[
 * \dot{V} = - \left({}^w\dot{R}_b{}^b\hat{V}\right)^\top e_3,
 * \f]
 * where \f$ {}^w\dot{R}_b = S({}^w\omega){}^w{R}_b \f$, with \f$ S(\cdot) \f$ being a skew symmetric matrix,
 * and \f$ {}^w\omega \in \mathbb{R}^3 \f$ is the right-trivialized angular velocity. Hence,
 * \f[
 * \dot{V} = - \left(S({}^w\omega){}^w{R}_b{}^b\hat{V}\right)^\top e_3.
 * \f]
 *
 * Exploiting \f$ S(\cdot)^\top = -S(\cdot) \f$, we have:
 * \f[
 * \dot{V} = {}^b\hat{V}^\top{}^w{R}_b^\top  S({}^w\omega) e_3,
 * \f]
 * and \f$ S(a)b = -S(b)a \f$:
 * \f[
 * \dot{V} = -{}^b\hat{V}^\top{}^w{R}_b^\top  S(e_3){}^w\omega.
 * \f]
 *
 * Then, if we choose:
 * \f[
 * {}^w\omega = k \left({}^b\hat{V}^\top{}^w{R}_b^\top  S(e_3)\right)^\top + \lambda e_3,
 * \f]
 * with \f$ k, \lambda \in \mathbb{R}^3 \f$, we have that \f$ \dot{V} \leq 0 \f$. Then,
 * \f[
 * {}^w\omega = -k S(e_3) {}^w{R}_b{}^b\hat{V} + \lambda e_3.
 * \f]
 *
 * We can introduce the right-trivialized Jacobian \f$ {}^wJ \f$ such that \f$ {}^wJ\dot{\nu} = {}^w\omega \f$:
 * \f[
 * {}^wJ\dot{\nu} =  -k S(e_3) {}^w{R}_b{}^b\hat{V} + \lambda e_3.
 * \f]
 *
 * We can filter out the rotation around the world Z-axis, thus obtaining:
 * \f[
 * \begin{bmatrix}
 *   e_1 & e_2
 * \end{bmatrix}^\top {}^wJ\dot{\nu} = -k \begin{bmatrix}
 *                                              0 & -1 & 0\\
 *                                              1 & 0 & 0
 *                                           \end{bmatrix} {}^w{R}_b{}^b\hat{V}.
 * \f]
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
     * @return True in case of success, false otherwise.
     *
     * The input is normalized, unless the norm is too small.
     */
    bool setDesiredGravityDirectionInTargetFrame(
        const Eigen::Ref<const Eigen::Vector3d> desiredGravityDirection);

    /**
     * @brief Set the feedforward angular velocity expressed in the target frame.
     * @param feedforwardVelocity The desired feedforward velocity in rad/s.
     * @return True in case of success, false otherwise.
     * Only the first two components are used.
     */
    bool
    setFeedForwardVelocityInTargetFrame(const Eigen::Ref<const Eigen::Vector3d> feedforwardVelocity);

    /**
     * @brief Set the desired gravity direction expressed the target frame and the feedforward
     * velocity.
     * @param desiredGravityDirection The desired gravity direction in target frame.
     * @param feedforwardVelocity The desired feedforward velocity in rad/s expressed in the target
     * frame.
     * @return True in case of success, false otherwise.
     *
     * The desiredGravityDirection is normalized, unless the norm is too small.
     * Only the first two components of the feedforwardVelocity are used.
     * This is equivalent of using setDesiredGravityDirectionInTargetFrame and
     * setFeedForwardVelocityInTargetFrame
     */
    bool setSetPoint(const Eigen::Ref<const Eigen::Vector3d> desiredGravityDirection,
                     const Eigen::Ref<const Eigen::Vector3d> feedforwardVelocity
                     = Eigen::Vector3d::Zero());

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
