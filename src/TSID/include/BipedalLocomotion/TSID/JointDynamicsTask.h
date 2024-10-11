/**
 * @file JointDynamicsTask.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TSID_JOINT_DYNAMICS_TASK_H
#define BIPEDAL_LOCOMOTION_TSID_JOINT_DYNAMICS_TASK_H

#include <vector>
#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <iDynTree/KinDynComputations.h>

namespace BipedalLocomotion
{
namespace TSID
{

/**
 * JointsDynamicsTask is a concrete implementation of the Task. Please use this element if you
 * want to consider the joints dynamics only.
 * The task represents the following equation
 * \f[
 * M_{s}(q) \dot{\nu} + h_{s}(q,\nu) = \sum J_k^\top f_k + \tau
 * \f]
 * where the suffix \f$s\f$ indicates the last n rows of the vector/matrix. \f$J_k\f$ is the robot
 * jacobian associated to the contact wrench \f$d_k\f$. \f$\tau\f$ are the joints torque.
 * @note Please consider that here the MIXED representation is used to define the 6d-velocity and
 * 6d-forces. You can find further details in Section 2.3.4 of
 * https://traversaro.github.io/phd-thesis/traversaro-phd-thesis.pdf.
 */
class JointDynamicsTask : public TSIDLinearTask
{
    System::VariablesHandler::VariableDescription m_robotAccelerationVariable; /**< Variable
                                                                                  describing the
                                                                                  robot acceleration
                                                                                  (base + joint) */

    System::VariablesHandler::VariableDescription m_jointsTorqueVariable; /**< Variable
                                                                             describing the joint
                                                                             torques */
    bool m_useMassMatrixRegularizationTerm{false}; /**< True if the mass matrix regularization is used*/
    Eigen::MatrixXd m_massMatrixRegularizationTerm; /**< Variable storing mass matrix regularization term*/
    struct ContactWrench
    {
        iDynTree::FrameIndex frameIndex; /**< Frame used to express the contact wrench */
        System::VariablesHandler::VariableDescription variable; /**< Variable describing the contact
                                                                   wrench */
    };

    std::vector<ContactWrench> m_contactWrenches; /**< List of the information related to the
                                                     contact wrenches */

    Eigen::MatrixXd m_massMatrix; /**< Mass Matrix */
    Eigen::MatrixXd m_jacobian; /**< Temporary jacobian */
    Eigen::VectorXd m_generalizedBiasForces; /**< Vector containing the generalized bias forces */

    static constexpr std::size_t m_spatialVelocitySize{6}; /**< Size of the spatial velocity vector
                                                            */

    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

public:
    /**
     * Initialize the planner.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                                    Description                                                   | Mandatory |
     * |:----------------------------------:|:--------:|:----------------------------------------------------------------------------------------------------------------:|:---------:|
     * | `robot_acceleration_variable_name` | `string` |              Name of the variable contained in `VariablesHandler` describing the robot acceleration              |    Yes    |
     * |    `joint_torques_variable_name`   | `string` |                 Name of the variable contained in `VariablesHandler` describing the joints torque                |    Yes    |
     * |      `max_number_of_contacts`      |   `int`  |                               Maximum number of contacts. The default value is 0.                                |     No    |
     * |            `CONTACT_<i>`           |  `group` | `i` is an `int` between `0` and `max_number_of_contacts` The group must contain `variable_name` and `frame_name` |    Yes    |
     * |           `variable_name`          | `string` |                    Name of the variable contained in `VariablesHandler` describing the contact                   |    Yes    |
     * |            `frame_name`            | `string` |                                    Name of the frame associated to the contact                                   |    Yes    |
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
     * @note The handler must contain variables named as the parameter
     * `robot_acceleration_variable_name`, `variable_name` (in `CONTACT_<i>` group) and
     * `joint_torques_variable_name`    stored in the parameter handler.
     * `robot_acceleration_variable_name` represents the generalized acceleration of the robot.
     * Where the generalized robot acceleration is a vector containing the base spatial-acceleration
     * (expressed in mixed representation) and the joints acceleration.
     * @return True in case of success, false otherwise.
     */
    bool setVariablesHandler(const System::VariablesHandler& variablesHandler) override;

    /**
     * Set the mass matrix regularization term. i.e. \f$\bar{M} = M + M _ {reg}\f$. Where  \f$M\f$
     * is the mass matrix and  \f$M_{reg}\f$ is the matrix regularization term.
     * @param matrix the regularization term for the mass matrix.
     * @notice Calling this function is not mandatory. Call it only if you want to add a
     * regularization term.
     * @return true in case of success, false otherwise.
     */
    bool setMassMatrixRegularization(const Eigen::Ref<const Eigen::MatrixXd>& matrix);

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
     * The JointDynamicsTask is an equality task.
     * @return the type of the task.
     */
    Type type() const override;

    /**
     * Determines the validity of the objects retrieved with getA() and getB()
     * @return True if the objects are valid, false otherwise.
     */
    bool isValid() const override;
};

BLF_REGISTER_TSID_TASK(JointDynamicsTask);

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_JOINT_DYNAMICS_TASK_H
