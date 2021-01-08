/**
 * @file SE3Task.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_TSID_OPTIMAL_SE3_TASK_H
#define BIPEDAL_LOCOMOTION_TSID_OPTIMAL_SE3_TASK_H

#include <manif/manif.h>

#include <BipedalLocomotion/TSID/OptimalControlElement.h>

#include <LieGroupControllers/ProportionalDerivativeController.h>

namespace BipedalLocomotion
{
namespace TSID
{

/**
 * SE3Task is a concrete implementation of the OptimalControlElement. Please use this element if you
 * want to control the position and the orientation of a given frame rigidly attached to the robot.
 * The task assumes perfect control of the robot acceleration \f$\dot{\nu}\f$ that contains the base
 * linear and angular acceleration expressed in mixed representation and the joints acceleration.
 * The task represents the following equation
 * \f[
 * J \dot{\nu} + \dot{J} \nu = \dot{\mathrm{v}} ^ *
 * \f]
 * where \f$J\f$ is the robot jacobian and \f$\dot{\mathrm{v}} ^ *\f$ is the desired acceleration.
 * The desired acceleration is chosen such that the frame will asymptotically converge to the
 * desired trajectory. The linear component of \f$\dot{\mathrm{v}} ^ *\f$ is computed with a
 * standard PD controller in \f$R^3\f$ while the angular acceleration is computed by a PD controller
 * in \f$SO(3)\f$.
 * @note Please refer to https://github.com/dic-iit/lie-group-controllers if you are interested in
 * the implementation of the PD controllers.
 */
class SE3Task : public OptimalControlElement
{
    LieGroupControllers::ProportionalDerivativeControllerSO3d m_SO3Controller; /**< PD Controller in
                                                                                  SO(3) */
    LieGroupControllers::ProportionalDerivativeControllerR3d m_R3Controller; /**< PD Controller in
                                                                                  R3 */

    System::VariablesHandler::VariableDescription m_robotAccelerationVariable; /**< Variable
                                                                                  describing the
                                                                                  robot acceleration
                                                                                  (base + joint) */

    iDynTree::FrameIndex m_frameIndex; /**< Frame controlled by the OptimalControlElement */
    Eigen::MatrixXd m_jacobian; /**< Jacobian matrix (here the jacobian is described by the so
                                   called MIXED representation) */

public:
    /**
     * Initialize the planner.
     * @param paramHandler pointer to the parameters handler.
     * @param variablesHandler reference to a variables handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                      | Mandatory |
     * |:----------------------------------:|:--------:|:--------------------------------------------------------------------------------------:|:---------:|
     * | `robot_acceleration_variable_name` | `string` | Name of the variable contained in `VariablesHandler` describing the robot acceleration |    Yes    |
     * |            `frame_name`            | `string` |                       Name of the farme controlled by the SE3Task                      |    Yes    |
     * |             `kp_linear`            | `double` |                             Gain of the position controller                            |    Yes    |
     * |             `kd_linear`            | `double` |                         Gain of the linear velocity controller                         |    Yes    |
     * |            `kp_angular`            | `double` |                           Gain of the orientation controller                           |    Yes    |
     * |            `kd_angular`            | `double` |                         Gain of the angular velocity controller                        |    Yes    |
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
     * @param I_H_F Homogeneous transform between the link and the inertial frame.
     * @param mixedVelocity 6D-velocity written in mixed representation.
     * @param mixedAcceleration 6D-acceleration written in mixed representation.
     * @return True in case of success, false otherwise.
     */
    bool setReferenceTrajectory(const manif::SE3d& I_H_F,
                                const manif::SE3d::Tangent& mixedVelocity,
                                const manif::SE3d::Tangent& mixedAcceleration);
};

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_OPTIMAL_SE3_TASK_H
