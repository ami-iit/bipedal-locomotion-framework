/**
 * @file AngularMomentumTask.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_ANGULAR_MOMENTUM_TASK_H
#define BIPEDAL_LOCOMOTION_IK_ANGULAR_MOMENTUM_TASK_H

#include <memory>

#include <BipedalLocomotion/IK/IKLinearTask.h>

#include <iDynTree/KinDynComputations.h>

namespace BipedalLocomotion
{
namespace IK
{

/**
 * AngularMomentumTask is a concrete implementation of the Task. Please use this element if you
 * want to control robot centroidal angular momentum.
 * The task assumes perfect control of the robot velocity \f$\nu\f$ that contains the base
 * linear and angular velocity expressed in mixed representation and the joint velocities.
 * The task represents the following equation
 * \f[
 * A_{cmm_\omega} \nu = {}^{G[A]} h_\omega ^*
 * \f]
 * where \f$A_{cmm_\omega}\f$ is the angular part of the robot centroidal momentum matrix
 * \f${}^{G[A]} h_\omega ^*\f$ is the desired centroidal angular momentum.
 * @note AngularMomentumTask can be used to control also a subset of element of the AngularMomentum.
 * Please refer to `mask` parameter in IK::AngularMomentumTask::initialize method.
 */
class AngularMomentumTask : public IKLinearTask
{
    System::VariablesHandler::VariableDescription m_robotVelocityVariable; /**< Variable
                                                                                  describing the
                                                                                  robot velocity
                                                                                  (base + joint) */

    static constexpr std::size_t m_angularVelocitySize{3}; /**< Size of the angular velocity vector.
                                                            */
    static constexpr std::size_t m_spatialVelocitySize{6}; /**< Size of the spatial velocity vector.
                                                            */

    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

    Eigen::MatrixXd m_centroidalMomentumMatrix;
    std::size_t m_angularMomentumTaskSize{m_angularVelocitySize}; /**< DoFs associated to the linear task */
    std::array<bool, m_angularVelocitySize> m_mask{true, true, true};
public:

    /**
     * Initialize the task.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                      | Mandatory |
     * |:----------------------------------:|:--------:|:--------------------------------------------------------------------------------------:|:---------:|
     * |    `robot_velocity_variable_name`  | `string` |   Name of the variable contained in `VariablesHandler` describing the robot velocity   |    Yes    |
     * |               `mask`               | `vector<bool>` |  Mask representing the angular momentum coordinates controlled. E.g. [1,0,1] will enable the control on the x and z coordinates only and the angular part. (Default value, [1,1,1])   |    No    |
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
    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

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
     * Set the desired reference trajectory.
     * @param desired angular momentum
     * @return True in case of success, false otherwise.
     */
    bool setSetPoint(Eigen::Ref<const Eigen::Vector3d> desiredAngularMomentumComponents);

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;

    /**
     * The AngularMomentumTask is an equality task.
     * @return the size of the task.
     */
    Type type() const override;

    /**
     * Determines the validity of the objects retrieved with getA() and getB()
     * @return True if the objects are valid, false otherwise.
     */
    bool isValid() const override;
};

} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_IK_ANGULAR_MOMENTUM_TASK_H
