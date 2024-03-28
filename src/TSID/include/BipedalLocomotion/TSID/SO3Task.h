/**
 * @file SO3Task.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TSID_SO3_TASK_H
#define BIPEDAL_LOCOMOTION_TSID_SO3_TASK_H

#include <manif/manif.h>

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

#include <iDynTree/KinDynComputations.h>

#include <LieGroupControllers/ProportionalDerivativeController.h>

namespace BipedalLocomotion
{
namespace TSID
{

/**
 * SO3Task is a concrete implementation of the TSIDLinearTask. Please use this element if you
 * want to control the orientation of a given frame rigidly attached to the robot.
 * The task assumes perfect control of the robot acceleration \f$\dot{\nu}\f$ that contains the base
 * linear and angular acceleration expressed in mixed representation and the joints acceleration.
 * The task represents the following equation
 * \f[
 * J \dot{\nu} + \dot{J} \nu = \dot{\mathrm{v}} ^ *
 * \f]
 * where \f$J\f$ is the robot jacobian and \f$\dot{\mathrm{v}} ^ *\f$ is the desired acceleration.
 * The desired acceleration is chosen such that the frame orientation will asymptotically converge
 * to the desired trajectory. The angular acceleration is computed by a PD controller in
 * \f$SO(3)\f$.
 * @note Please refer to https://github.com/ami-iit/lie-group-controllers if you are interested in
 * the implementation of the PD controllers.
 */
class SO3Task : public TSIDLinearTask
{
    LieGroupControllers::ProportionalDerivativeControllerSO3d m_SO3Controller; /**< PD Controller in
                                                                                  SO(3) */

    System::VariablesHandler::VariableDescription m_robotAccelerationVariable; /**< Variable
                                                                                  describing the
                                                                                  robot acceleration
                                                                                  (base + joint) */

    iDynTree::FrameIndex m_frameIndex; /**< Frame controlled by the task */

    static constexpr std::size_t m_angularVelocitySize{3}; /**< Size of the angular velocity vector. */
    static constexpr std::size_t m_spatialVelocitySize{6}; /**< Size of the spatial velocity vector. */

    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

    Eigen::MatrixXd m_jacobian; /**< Jacobian of the frame expressed in mixed representation */

public:
    /**
     * Initialize the SO3Task.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                      | Mandatory |
     * |:----------------------------------:|:--------:|:--------------------------------------------------------------------------------------:|:---------:|
     * | `robot_acceleration_variable_name` | `string` | Name of the variable contained in `VariablesHandler` describing the robot acceleration |    Yes    |
     * |            `frame_name`            | `string` |                       Name of the frame controlled by the SO3Task                      |    Yes    |
     * |            `kp_angular`            | `double` or `vector<double>` |                           Gain of the orientation controller                           |    Yes    |
     * |            `kd_angular`            | `double` or `vector<double>` |                         Gain of the angular velocity controller                        |    Yes    |
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
     * the generalized acceleration of the robot, where the generalized robot acceleration is a
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
     * Set the desired reference point.
     * @param I_R_F Rotation between the link and the inertial frame.
     * @param angularVelocity angular velocity of the frame F wr.t. the inertial frame expressed in
     * the inertial frame.
     * @param angularAcceleration angular acceleration of the frame F wr.t. the inertial frame
     * expressed in the inertial frame.
     * @return True in case of success, false otherwise.
     */
    bool setSetPoint(const manif::SO3d& I_R_F,
                     const manif::SO3d::Tangent& angularVelocity,
                     const manif::SO3d::Tangent& angularAcceleration);

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;

    /**
     * The SO3Task is an equality task.
     * @return the type of the task.
     */
    Type type() const override;

    /**
     * Determines the validity of the objects retrieved with getA() and getB()
     * @return True if the objects are valid, false otherwise.
     */
    bool isValid() const override;
};

BLF_REGISTER_TSID_TASK(SO3Task);

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_SO3_TASK_H
