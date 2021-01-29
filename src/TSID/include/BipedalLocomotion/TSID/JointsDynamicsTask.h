/**
 * @file JointsDynamicsTask.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_TSID_JOINTS_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_TSID_JOINTS_DYNAMICS_H

#include <BipedalLocomotion/TSID/Task.h>

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
 * jacobian associated to the contact wrench \f$d_k\f$. \f$\tau$\f are the joints torque.
 * @note Please consider that here the MIXED representation is used to define the 6d-velocity and
 * 6d-forces. You can find further details in Section 2.3.4 of
 * https://traversaro.github.io/phd-thesis/traversaro-phd-thesis.pdf.
 */
class JointsDynamicsTask : public Task
{
    System::VariablesHandler::VariableDescription m_robotAccelerationVariable; /**< Variable
                                                                                  describing the
                                                                                  robot acceleration
                                                                                  (base + joint) */

    System::VariablesHandler::VariableDescription m_jointsTorqueVariable; /**< Variable
                                                                             describing the joint
                                                                             torques */

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

public:
    /**
     * Initialize the planner.
     * @param paramHandler pointer to the parameters handler.
     * @param variablesHandler reference to a variables handler.
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
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> paramHandler,
                    const System::VariablesHandler& variablesHandler) override;

    /**
     * Update the content of the element.
     * @return True in case of success, false otherwise.
     */
    bool update() override;
};

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_JOINTS_DYNAMICS_H
