/**
 * @file CoMTask.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TSID_COM_TASK_H
#define BIPEDAL_LOCOMOTION_TSID_COM_TASK_H

#include <memory>

#include <manif/manif.h>

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

#include <iDynTree/KinDynComputations.h>

#include <LieGroupControllers/ProportionalDerivativeController.h>

namespace BipedalLocomotion
{
namespace TSID
{

/**
 * ComTask is a concrete implementation of the TSIDLinearTask. Please use this element if you
 * want to control the position of the CoM. The task assumes perfect control of the robot
 * acceleration \f$\dot{\nu}\f$ that contains the base linear and angular acceleration expressed in
 * mixed representation and the joints acceleration. The task represents the following equation
 * \f[
 * J \dot{\nu} + \dot{J} \nu = \dot{v}_{CoM} ^ *
 * \f]
 * where \f$J\f$ is the robot CoM jacobian and \f$\dot{v}_{CoM} ^ *\f$ is the desired acceleration.
 * The desired acceleration is chosen such that the CoM will asymptotically converge to the
 * desired trajectory. \f$\dot{v} ^ *\f$ is computed with a
 * standard PD controller in \f$R^3\f$.
 * @note Please refer to https://github.com/ami-iit/lie-group-controllers if you are interested in
 * the implementation of the PD controllers.
 */
class CoMTask : public TSIDLinearTask
{
    LieGroupControllers::ProportionalDerivativeControllerR3d m_R3Controller; /**< P Controller in R3
                                                                              */

    System::VariablesHandler::VariableDescription m_robotAccelerationVariable; /**< Variable
                                                                                  describing the
                                                                                  robot acceleration
                                                                                  (base + joint) */

    static constexpr std::size_t m_linearVelocitySize{3}; /**< Size of the linear velocity vector. */
    static constexpr std::size_t m_spatialVelocitySize{6}; /**< Size of the spatial velocity vector.*/

    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

    /** Mask used to select the DoFs controlled by the task */
    std::array<bool, m_linearVelocitySize> m_mask{true, true, true};
    std::size_t m_DoFs{m_linearVelocitySize}; /**< DoFs associated to the task */
    Eigen::MatrixXd m_jacobian; /**< CoM Jacobian matrix in MIXED representation */
public:

    /**
     * Initialize the task.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |               Type           |                                       Description                                      | Mandatory |
     * |:----------------------------------:|:----------------------------:|:--------------------------------------------------------------------------------------:|:---------:|
     * | `robot_acceleration_variable_name` |            `string`          | Name of the variable contained in `VariablesHandler` describing the robot acceleration |    Yes    |
     * |             `kp_linear`            | `double` or `vector<double>` |                             Gain of the position controller                            |    Yes    |
     * |             `kd_linear`            | `double` or `vector<double>` |                         Gain of the linear velocity controller                         |    Yes    |
     * |               `mask`               |         `vector<bool>`       |  Mask representing the DoFs controlled. E.g. [1,0,1] will enable the control on the x and z coordinates only. (Default value, [1,1,1])   |    No     |
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
     * Set the desired set-point of the trajectory.
     * @param position position of the CoM
     * @param velocity velocity of the CoM expressed in mixed representation
     * @param acceleration acceleration of the CoM expressed in mixed representation
     * @return True in case of success, false otherwise.
     */
    bool setSetPoint(Eigen::Ref<const Eigen::Vector3d> position,
                     Eigen::Ref<const Eigen::Vector3d> velocity = Eigen::Vector3d::Zero(),
                     Eigen::Ref<const Eigen::Vector3d> acceleration = Eigen::Vector3d::Zero());

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;

    /**
     * The CoMTask is an equality task.
     * @return the size of the task.
     */
    Type type() const override;

    /**
     * Determines the validity of the objects retrieved with getA() and getB()
     * @return True if the objects are valid, false otherwise.
     */
    bool isValid() const override;
};

BLF_REGISTER_TSID_TASK(CoMTask);

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_COM_TASK_H
