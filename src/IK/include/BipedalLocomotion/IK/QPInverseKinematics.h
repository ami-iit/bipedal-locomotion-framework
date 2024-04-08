/**
 * @file QPInverseKinematics.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_QP_INVERSE_KINEMATICS_H
#define BIPEDAL_LOCOMOTION_IK_QP_INVERSE_KINEMATICS_H

#include <memory>
#include <vector>
#include <string>

#include <Eigen/Dense>

#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/System/ILinearTaskSolver.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

namespace BipedalLocomotion
{

namespace IK
{

/**
 * QPInverseKinematics is a concrete class and implements an integration base inverse kinematics.
 * The inverse kinematics is here implemented as Quadratic Programming (QP) problem. The user should
 * set the desired task with the method QPInverseKinematics::addTask. Each task has a given
 * priority. Currently we support only priority equal to 0 or 1. If the task priority is set to 0
 * the task will be considered as hard task, thus treated as an equality constraint. If the priority
 * is equal to 1 the task will be embedded in the cost function. The class is also able to treat
 * inequality constraints.
 * A possible usage of the IK can be found in "Romualdi et al. A Benchmarking of DCM Based
 * Architectures for Position and Velocity Controlled Walking of Humanoid Robots"
 * https://doi.org/10.1109/HUMANOIDS.2018.8625025
 * Here you can find an example of the QPInverseKinematics class used as velocity controller or IK
 * @subsection qp_vc Velocity Control
 * Here you can find an example of the QPFixedBaseInverseKinematics interface used as
 * a velocity controller.
 * <br/>
 * <img src="https://user-images.githubusercontent.com/16744101/142453785-9e6f2b5e-dc82-417a-a5e3-bc8c61865d0b.png" alt="VelocityControl" width="1500">
 * @subsection qp_ik Inverse Kinematics
 * If you want to use QPInverseKinematics as IK you need to integrate the output
 * velocity. System::FloatingBaseSystemKinematics and System::Integrator classes can be used
 * to integrate the output of the IK taking into account the geometrical structure of the
 * configuration space (\f$ \mathbb{R}^3 \times SO(3) \times \mathbb{R}^n\f$)
 * <br/>
 * <img src="https://user-images.githubusercontent.com/16744101/142453860-6bba2a7a-26af-48da-b04e-114314c6f67c.png" alt="InverseKinematics" width="1500">
 */
class QPInverseKinematics : public IntegrationBasedIK
{
    /**
     * Private implementation
     */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:

    /**
     * Constructor.
     */
    QPInverseKinematics();

    /**
     * Destructor.
     */
    virtual ~QPInverseKinematics();

    /**
     * Add a linear task in the solver.
     * @param task pointer to a given linear task
     * @param taskName unique name associated to the task.
     * @param priority Priority associated to the task. The lower the number the higher the
     * priority.
     * @param weightProvider Weight provider associated to the task. This parameter is optional. The
     * default value is an object that does not contain any value. The user may avoid to pass a
     * provider only if the priority of the task is equal to 0.
     * @return true if the task has been added to the solver.
     * @warning The QPTSID cannot handle inequality tasks (please check Task::Type) with priority
     * equal to 1.
     * @warning The QPTSID can handle only priority equal to 0 and 1. 0 means high priority while 1
     * low priority.
     */
    bool
    addTask(std::shared_ptr<Task> task,
            const std::string& taskName,
            std::size_t priority,
            std::shared_ptr<const System::WeightProviderPort> weightProvider = nullptr) override;

    /**
     * Add a linear task in the solver.
     * @param task pointer to a given linear task
     * @param taskName unique name associated to the task.
     * @param priority Priority associated to the task. The lower the number the higher the
     * priority.
     * @param weight Weight associated to the task.
     * @return true if the task has been added to the solver.
     * @note The solver assumes the weight is a constant value.
     * @warning The QPInverseKinematics cannot handle inequality tasks (please check Task::Type)
     * with priority equal to 1.
     */
    bool addTask(std::shared_ptr<Task> task,
                 const std::string& taskName,
                 std::size_t priority,
                 Eigen::Ref<const Eigen::VectorXd> weight) override;

    /**
     * Set the weightProvider associated to an already existing task
     * @param taskName name associated to the task
     * @param weightProvider new weight provider associated to the task.
     * @return true if the weight has been updated
     */
    bool setTaskWeight(const std::string& taskName,
                       std::shared_ptr<const System::WeightProviderPort> weightProvider) override;

    /**
     * Set the weight associated to an already existing task
     * @param taskName name associated to the task
     * @param weight new Weight associated to the task. A constant weight is assumed.
     * @return true if the weight has been updated
     */
    bool setTaskWeight(const std::string& taskName,
                       Eigen::Ref<const Eigen::VectorXd> weight) override;

    /**
     * Get the weightProvider associated to an already existing task
     * @param taskName name associated to the task
     * @return a weak pointer to the weightProvider. If the task does not exist the pointer is not
     * lockable
     */
    std::weak_ptr<const System::WeightProviderPort>
    getTaskWeightProvider(const std::string& taskName) const override;

    /**
     * Finalize the IK.
     * @param handler parameter handler.
     * @note You should call this method after you add ALL the tasks.
     * @return true in case of success, false otherwise.
     */
    bool finalize(const System::VariablesHandler& handler) override;


    /**
     * Initialize the inverse kinematics algorithm.
     * @param handler pointer to the IParametersHandler interface.
     * @note the following parameters are required by the class
     * |         Parameter Name         |   Type   |                                           Description                                          | Mandatory |
     * |:------------------------------:|:--------:|:----------------------------------------------------------------------------------------------:|:---------:|
     * | `robot_velocity_variable_name` | `string` | Name of the variable contained in `VariablesHandler` describing the generalized robot velocity |    Yes    |
     * |           `verbosity`          |  `bool`  |                         Verbosity of the solver. Default value `false`                         |     No    |
     * |           `precision`             | `double` |                          Precision of the solver. Default value `1e-6`                         |     No    |
     * Where the generalized robot velocity is a vector containing the base spatial velocity
     * (expressed in mixed representation) and the joint velocities.
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    /**
     * Build the inverse kinematics solver
     * @param kinDyn a pointer to an iDynTree::KinDynComputations object that will be shared among
     * all the tasks.
     * @param handler pointer to the IParametersHandler interface.
     * @note the following parameters are required by the class
     * |   Group   |         Parameter Name         |       Type      |                                           Description                                          | Mandatory |
     * |:---------:|:------------------------------:|:---------------:|:----------------------------------------------------------------------------------------------:|:---------:|
     * |           |           `tasks`              | `vector<string>`|         Vector containing the list of the tasks considered in the IK.                          |    Yes    |
     * |   `IK`    | `robot_velocity_variable_name` |     `string`    | Name of the variable contained in `VariablesHandler` describing the generalized robot velocity |    Yes    |
     * |   `IK`    |           `verbosity`          |      `bool`     |                         Verbosity of the solver. Default value `false`                         |     No    |
     * Where the generalized robot velocity is a vector containing the base spatialvelocity
     * (expressed in mixed representation) and the joint velocities.
     * For **each** task listed in the parameter `tasks` the user must specify all the parameters
     * required by the task itself but `robot_velocity_variable_name` since is already specified in
     * the `IK` group. Moreover the following parameters are required for each task.
     * |   Group   |         Parameter Name         |       Type      |                                           Description                                          | Mandatory |
     * |:---------:|:------------------------------:|:---------------:|:----------------------------------------------------------------------------------------------:|:---------:|
     * |`TASK_NAME`|             `type`             |     `string`    |   String representing the type of the task. The string should match the name of the C++ class. |    Yes    |
     * |`TASK_NAME`|           `priority`           |       `int`     | Priority associated to the task.  (Check QPInverseKinematics::addTask for further information) |    Yes    |
     * |`TASK_NAME`|     `weight_provider_type`     |     `string`    |  String representing the type of the weight provider. The string should match the name of the C++ class. It is required only if the task is low priority. The default value in case of low priority task (`priority = 1`) is `ConstantWeightProvider`            |     No    |
     * Given the weight type specified by `weight_provider_type`, the user must specify all the
     * parameters required by the provider in the `TASK_NAME` group handler
     * `TASK_NAME` is a placeholder for the name of the task contained in the `tasks` list.
     * @note The following `ini` file presents an example of the configuration that can be used to
     * build the IK
     * ~~~~~{.ini}
     * tasks                           ("COM_TASK", "RIGHT_FOOT_TASK", "LEFT_FOOT_TASK", "TORSO_TASK", "JOINT_REGULARIZATION_TASK")
     *
     * [IK]
     * robot_velocity_variable_name    robot_velocity
     *
     * [COM_TASK]
     * type                            CoMTask
     * kp_linear                       2.0
     * mask                            (true, true, true)
     * priority                        0
     *
     * [RIGHT_FOOT_TASK]
     * type                            SE3Task
     * frame_name                      r_sole
     * kp_linear                       7.0
     * kp_angular                      5.0
     * priority                        0
     *
     * [LEFT_FOOT_TASK]
     * type                            SE3Task
     * frame_name                      l_sole
     * kp_linear                       7.0
     * kp_angular                      5.0
     * priority                        0
     *
     *
     * [JOINT_REGULARIZATION_TASK]
     * type                            JointTrackingTask
     * kp                              (5.0, 5.0, 5.0,
     *                                  5.0, 5.0, 5.0, 5.0,
     *                                  5.0, 5.0, 5.0, 5.0,
     *                                  5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
     *                                  5.0, 5.0, 5.0, 5.0, 5.0, 5.0)
     * priority                        1
     * weight_provider_type            ConstantWeightProvider
     * weight                          (1.0, 1.0, 1.0,
     *                                  2.0, 2.0, 2.0, 2.0,
     *                                  2.0, 2.0, 2.0, 2.0,
     *                                  1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
     *                                  1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
     *
     * [include TORSO_TASK "./tasks/torso.ini"]
     * ~~~~~
     * Where the file `./tasks/torso.ini` contains the definition of a low priority task whose
     * weight is a BipedalLocomotion::ContinuousDynamicalSystem::MultiStateWeightProvider.
     * Since `MultiStateWeightProvider` requires the definition of subgroups, an additional file is
     * suggested as explained in: https://github.com/robotology/yarp/discussions/2563
     * ~~~~~{.ini}
     * type                            SO3Task
     * frame_name                      chest
     * kp_angular                      5.0
     *
     * weight_provider_type            MultiStateWeightProvider
     *
     * states                          ("STANCE", "WALKING")
     * sampling_time                   0.01
     * settling_time                   3.0
     *
     * [STANCE]
     * name                            stance
     * weight                          (0.1, 0.1, 0.1)
     *
     * [WALKING]
     * name                            walking
     * weight                          (5.0, 5.0, 5.0)
     * ~~~~~
     * @return an IntegrationBasedIKProblem. In case of issues an invalid IntegrationBasedIKProblem
     * will be returned.
     */
    static IntegrationBasedIKProblem
    build(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
          std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    /**
     * Get a vector containing the name of the tasks.
     * @return an std::vector containing all the names associated to the tasks
     */
    std::vector<std::string> getTaskNames() const override;

    /**
     * Return true if the content of get is valid.
     */
    bool isOutputValid() const override;

    /**
     * Solve the inverse kinematics.
     * @return true in case of success and false otherwise.
     */
    bool advance() override;

    /**
     * Get the outcome of the optimization problem
     * @return the state of the inverse kinematics.
     */
    const State& getOutput() const override;

    /**
     * Get a specific task
     * @param name name associated to the task.
     * @return a weak ptr associated to an existing task in the IK. If the task does not exist a
     * nullptr is returned.
     */
    std::weak_ptr<Task> getTask(const std::string& name) const override;

    /**
     * Return the description of the InverseKinematics problem.
     * @return a string containing the description of the solver.
     */
    std::string toString() const override;

    /**
     * Return the vector representing the entire solution of the QPInverseKinematics.
     * @return a vector containing the solution of the optimization problem
     */
    Eigen::Ref<const Eigen::VectorXd> getRawSolution() const override;
};
} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_IK_QP_INVERSE_KINEMATICS_H
