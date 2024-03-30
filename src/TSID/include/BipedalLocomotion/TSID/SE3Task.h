/**
 * @file SE3Task.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TSID_SE3_TASK_H
#define BIPEDAL_LOCOMOTION_TSID_SE3_TASK_H

#include <manif/manif.h>

#include <BipedalLocomotion/System/ITaskControllerManager.h>
#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

#include <iDynTree/KinDynComputations.h>

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
 * @note Please refer to https://github.com/ami-iit/lie-group-controllers if you are interested in
 * the implementation of the PD controllers.
 * @note The SE3Task is technically not a \f$SE(3)\f$ space defined task, instead is a \f$SO(3)
 * \times \mathbb{R}^3\f$ task. Theoretically, there are differences between the two due to the
 * different definitions of exponential maps and logarithm maps. Please consider that here the MIXED
 * representation is used to define the 6d-velocity. You can find further details in Section 2.3.4
 * of https://traversaro.github.io/phd-thesis/traversaro-phd-thesis.pdf.
 */
class SE3Task : public TSIDLinearTask, public BipedalLocomotion::System::ITaskControllerManager
{
public:
    using Mode = System::ITaskControllerManager::Mode;

private:
    LieGroupControllers::ProportionalDerivativeControllerSO3d m_SO3Controller; /**< PD Controller in
                                                                                  SO(3) */
    LieGroupControllers::ProportionalDerivativeControllerR3d m_R3Controller; /**< PD Controller in
                                                                                  R3 */

    System::VariablesHandler::VariableDescription m_robotAccelerationVariable; /**< Variable
                                                                                  describing the
                                                                                  robot acceleration
                                                                                  (base + joint) */

    iDynTree::FrameIndex m_frameIndex; /**< Frame controlled by the OptimalControlElement */

    static constexpr std::size_t m_spatialVelocitySize{6}; /**< Size of the spatial velocity vector. */
    static constexpr std::size_t m_linearVelocitySize{3}; /**< Size of the linear velocity vector. */
    static constexpr std::size_t m_angularVelocitySize{3}; /**< Size of the angular velocity vector. */


    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

    /** Mask used to select the DoFs controlled by the task */
    std::array<bool, m_linearVelocitySize> m_mask{true, true, true};
    std::size_t m_linearDoFs{m_linearVelocitySize}; /**< DoFs associated to the linear task */
    std::size_t m_DoFs{m_spatialVelocitySize}; /**< DoFs associated to the entire task */

    Eigen::MatrixXd m_jacobian; /**< Jacobian matrix in MIXED representation */
    Eigen::VectorXd m_controllerOutput; /**< Controller output */

    /** State of the proportional derivative controller implemented in the task */
    Mode m_controllerMode{Mode::Enable};

public:
    /**
     * Initialize the planner.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                      | Mandatory |
     * |:----------------------------------:|:--------:|:--------------------------------------------------------------------------------------:|:---------:|
     * | `robot_acceleration_variable_name` | `string` | Name of the variable contained in `VariablesHandler` describing the robot acceleration |    Yes    |
     * |            `frame_name`            | `string` |                       Name of the frame controlled by the SE3Task                      |    Yes    |
     * |             `kp_linear`            | `double` or `vector<double>` |                             Gains of the position controller                            |    Yes    |
     * |             `kd_linear`            | `double` or `vector<double>` |                         Gains of the linear velocity controller                         |    Yes    |
     * |            `kp_angular`            | `double` or `vector<double>` |                           Gain of the orientation controller                           |    Yes    |
     * |            `kd_angular`            | `double` or `vector<double>` |                         Gain of the angular velocity controller                        |    Yes    |
     * |               `mask`               | `vector<bool>` |  Mask representing the linear DoFs controlled. E.g. [1,0,1] will enable the control on the x and z coordinates only and the angular part. (Default value, [1,1,1])   |    No    |
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
     * `robot_acceleration_variable_name` stored in the parameter handler. The variable represents
     * the generalized acceleration of the robot. Where the generalized robot acceleration is a
     * vector containing the base spatial-acceleration (expressed in mixed representation) and the
     * joints acceleration.
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
     * @param I_H_F Homogeneous transform between the link and the inertial frame.
     * @param mixedVelocity 6D-velocity written in mixed representation.
     * @param mixedAcceleration 6D-acceleration written in mixed representation.
     * @return True in case of success, false otherwise.
     */
    bool setSetPoint(const manif::SE3d& I_H_F,
                     const manif::SE3d::Tangent& mixedVelocity,
                     const manif::SE3d::Tangent& mixedAcceleration);

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;

    /**
     * The SE3Task is an equality task.
     * @return the type of the task.
     */
    Type type() const override;

    /**
     * Determines the validity of the objects retrieved with getA() and getB()
     * @return True if the objects are valid, false otherwise.
     */
    bool isValid() const override;

    /**
     * Set the task controller mode. Please use this method to disable/enable the Proportional
     * Derivative controller implemented in this task.
     * @param state state of the controller
     */
    void setTaskControllerMode(Mode mode) override;

    /**
     * Get the task controller mode.
     * @return the state of the controller
     */
    Mode getTaskControllerMode() const override;

    /**
     * Get the controller output.
     * @note the value of the controller output is changed by the update method.
     * @return a const reference to the controller output.
     */
    Eigen::Ref<const Eigen::VectorXd> getControllerOutput() const;

};

BLF_REGISTER_TSID_TASK(SE3Task);

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_SE3_TASK_H
