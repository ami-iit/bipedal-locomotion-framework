/**
 * @file IntegrationBasedIK.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_INTEGRATION_BASE_IK_H
#define BIPEDAL_LOCOMOTION_IK_INTEGRATION_BASE_IK_H

#include <memory>
#include <string>
#include <optional>

#include <Eigen/Dense>
#include <manif/SE3.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/IK/LinearTask.h>

namespace BipedalLocomotion
{

namespace IK
{

/**
 * State of the InverseKinematics
 */
struct IntegrationBasedIKState
{
    Eigen::VectorXd jointVelocity; /**< Joints velocity in rad per seconds */
    manif::SE3d::Tangent baseVelocity; /**< Mixed spatial velocity of the base */
};

/**
 * IntegrationBasedInverseKinematics implements the interface for the integration base inverse
 * kinematics. Please inherits this class if you want to implement your custom Integration base
 * Inverse Kinematics. The IntegrationBasedInverseKinematics can actually be used as Velocity
 * controller or real IK. Indeed it is important to notice that IntegrationBasedIKState is a struct
 * containing the joint velocities. When a robot velocity controller is available, one can set these
 * joint velocities to the low-level robot controller. In this case, the \f$t ^ d\f$ quantities in
 * the following figures  can be evaluated by using robot sensor feedback, and the robot is said to
 * be velocity controlled. On the other hand, if the robot velocity control is not available, one
 * may integrate the outcome of IntegrationBasedIK to obtain the desired joint position to be set to
 * a low-level robot position controller. In this case, the \f$t ^d\f$ quantities can be evaluated
 * by using the desired integrated quantities instead of sensor feedback, and the block behaves as
 * an inverse kinematics module, and the robot is said to be position controlled.
 * @subsection vc Velocity Control
 * Here you can find an example of the IntegrationBasedInverseKinematics interface used as
 * a velocity controller.
 * <br/>
 * <img src="https://user-images.githubusercontent.com/16744101/110701009-ec359200-81f0-11eb-9552-d47632f5b268.png" alt="VelocityControl" width="1500">
 * @subsection ik Inverse Kinematics
 * If you want to use IntegrationBasedInverseKinematics as IK you need to integrate the output
 * velocity. System::FloatingBaseSystemKinematics and System::Integrator classes can be used
 * to integrate the output of the IK taking into account the geometrical structure of the
 * configuration space (\f$ R^3 \times SO(3) \times R^n\f$)
 * <br/>
 * <img src="https://user-images.githubusercontent.com/16744101/110700993-e50e8400-81f0-11eb-88a1-30d5a024da9a.png" alt="InverseKinematics" width="1500">
 */
class IntegrationBasedIK : public BipedalLocomotion::System::Advanceable<IntegrationBasedIKState>
{

public:
    /**
     * Initialize the inverse kinematics algorithm.
     * @param handler pointer to the IParametersHandler interface.
     * @return true in case of success/false otherwise.
     */
    virtual bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler);

    /**
     * Add a linear task in the inverse kinematics
     * @param task pointer to a given linear task
     * @param priority Priority associated to the task. The lower the number the higher the
     * priority.
     * @param weight Weight associated to the task. This parameter is optional. The default value is
     * an object that does not contain any value. So is an invalid weight.
     * @return true if the task has been added to the inverse kinematics.
     */
    virtual bool addTask(std::shared_ptr<LinearTask> task,
                         const std::string& taskName,
                         std::size_t priority,
                         std::optional<Eigen::Ref<const Eigen::VectorXd>> weight = {}) = 0;

    /**
     * Get a vector containing the name of the tasks.
     * @return an std::vector containing all the names associated to the tasks
     */
    virtual std::vector<std::string> getTaskNames() const = 0;

    /**
     * Finalize the IK.
     * @param handler parameter handler.
     * @note You should call this method after you add ALL the tasks.
     * @return true in case of success, false otherwise.
     */
    virtual bool finalize(const System::VariablesHandler& handler) = 0;

    /**
     * Get a specific task
     * @param name name associated to the task.
     * @return a weak ptr associated to an existing task in the IK. If the task does not exist a
     * nullptr is returned.
     */
    virtual std::weak_ptr<LinearTask> getTask(const std::string& name) const = 0;

    /**
     * Destructor.
     */
    virtual ~IntegrationBasedIK() = default;
};
} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_IK_INTEGRATION_BASE_INVERSE_KINEMATICS_H
