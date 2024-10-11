/**
 * @file QPTSID.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TSID_QP_TSID_H
#define BIPEDAL_LOCOMOTION_TSID_QP_TSID_H

#include <functional>
#include <memory>
#include <optional>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/System/WeightProvider.h>
#include <BipedalLocomotion/TSID/TaskSpaceInverseDynamics.h>

#include <iDynTree/KinDynComputations.h>

namespace BipedalLocomotion
{

namespace TSID
{

/**
 * QPTSID is a concrete class and implements a task space inverse dynamics.
 * The TSID is here implemented as Quadratic Programming (QP) problem. The user should
 * set the desired task with the method QPTSID::addTask. Each task has a given
 * priority. Currently we support only priority equal to 0 or 1. If the task priority is set to 0
 * the task will be considered as hard task, thus treated as an equality constraint. If the priority
 * is equal to 1 the task will be embedded in the cost function. The class is also able to treat
 * inequality constraints.
 * A possible usage of the IK can be found in "Romualdi et al. A Benchmarking of DCM-Based
 * Architectures for Position, Velocity and Torque-Controlled Humanoid Robots"
 * https://doi.org/10.1142/S0219843619500348
 */
class QPTSID : public TaskSpaceInverseDynamics
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
    QPTSID();

    /**
     * Destructor.
     */
    virtual ~QPTSID();

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
     * @warning The QPTSID cannot handle inequality tasks (please check Task::Type) with priority
     * equal to 1.
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
    bool
    setTaskWeight(const std::string& taskName, Eigen::Ref<const Eigen::VectorXd> weight) override;

    /**
     * Get the weightProvider associated to an already existing task
     * @param taskName name associated to the task
     * @return a weak pointer to the weightProvider. If the task does not exist the pointer is not
     * lockable
     */
    std::weak_ptr<const System::WeightProviderPort>
    getTaskWeightProvider(const std::string& taskName) const override;

    /**
     * Get a vector containing the name of the tasks.
     * @return an std::vector containing all the names associated to the tasks
     */
    std::vector<std::string> getTaskNames() const override;

    /**
     * Get a specific task
     * @param name name associated to the task.
     * @return a weak ptr associated to an existing task in the TSID. If the task does not exist a
     * nullptr is returned.
     */
    std::weak_ptr<Task> getTask(const std::string& name) const override;

    // clang-format off
    /**
     * Initialize the TSID algorithm.
     * @param handler pointer to the IParametersHandler interface.h
     * @note the following parameters are required by the class
     * |            Parameter Name            |       Type       |                                             Description                                            | Mandatory |
     * |:------------------------------------:|:----------------:|:--------------------------------------------------------------------------------------------------:|:---------:|
     * |  `robot_acceleration_variable_name`  |     `string`     | Name of the variable contained in `VariablesHandler` describing the generalized robot acceleration |    Yes    |
     * |    `joint_torques_variable_name`      |     `string`     |         Name of the variable contained in `VariablesHandler` describing the robot torque           |    Yes    |
     * |  `contact_wrench_variables_name`     | `vector<string>` |        List of the variables associated to the contact wrenches in the `VariablesHandler`          |    Yes    |
     * |             `verbosity`              |      `bool`      |                        Verbosity of the solver. Default value `false`                              |     No    |
     * Where the generalized robot acceleration is a vector containing the base acceleration
     * (expressed in mixed representation) and the joint accelerations,
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;
    // clang-format on

    /**
     * Finalize the TSID.
     * @param handler parameter handler.
     * @note You should call this method after you add ALL the tasks.
     * @return true in case of success, false otherwise.
     */
    bool finalize(const System::VariablesHandler& handler) override;

    /**
     * Solve the fixed base TSID.
     * @return true in case of success and false otherwise.
     */
    bool advance() override;

    /**
     * Get the outcome of the optimization problem
     * @return the state of the TSID.
     */
    const State& getOutput() const override;

    /**
     * Return true if the content of get is valid.
     */
    bool isOutputValid() const override;

    /**
     * Return the description of the TSID problem.
     */
    std::string toString() const override;

    /**
     * Return the vector representing the entire solution of the QPTSID.
     * @return a vector containing the solution of the optimization problem
     */
    Eigen::Ref<const Eigen::VectorXd> getRawSolution() const override;

    // clang-format off
    /**
     * Build QPTSID problem
     * @param kinDyn a pointer to an iDynTree::KinDynComputations object that will be shared among
     * all the tasks.
     * @param handler pointer to the IParametersHandler interface.
     * @note the following parameters are required by the class
     * |   Group   |            Parameter Name          |       Type      |                                                      Description                                             | Mandatory |
     * |:---------:|:----------------------------------:|:---------------:|:------------------------------------------------------------------------------------------------------------:|:---------:|
     * |           |              `tasks`               | `vector<string>`|                 Vector containing the list of the tasks considered in the TSID.                              |    Yes    |
     * |  `TSID`   | `robot_acceleration_variable_name` |     `string`    |    Name of the variable contained in `VariablesHandler` representing the generalized robot acceleration.     |    Yes    |
     * |  `TSID`   |    `joint_torques_variable_name`   |     `string`    |              Name of the variable contained in `VariablesHandler` representing the joints torque.            |    Yes    |
     * |  `TSID`   |   `contact_wrench_variables_name`  | `vector<string>`| Vector containing the names  of the variable contained in `VariablesHandler` representing the contact wrench.|    Yes    |
     * |  `TSID`   |             `verbosity`            |      `bool`     |                         Verbosity of the solver. Default value `false`                                       |     No    |
     * Where the generalized robot acceleration is a vector containing the base spatial acceleration
     * (expressed in mixed representation) and the joint acceleration.
     * For **each** task listed in the parameter `tasks` the user must specify all the parameters
     * required by the task itself but `robot_acceleration_variable_name` and `joint_torques_variable_name`
     * since is already specified in the `IK` group. Moreover the following parameters are required for each task.
     * |   Group   |         Parameter Name         |       Type      |                                           Description                                          | Mandatory |
     * |:---------:|:------------------------------:|:---------------:|:----------------------------------------------------------------------------------------------:|:---------:|
     * |`TASK_NAME`|             `type`             |     `string`    |   String representing the type of the task. The string should match the name of the C++ class. |    Yes    |
     * |`TASK_NAME`|           `priority`           |       `int`     | Priority associated to the task.  (Check QPInverseKinematics::addTask for further information) |    Yes    |
     * |`TASK_NAME`|     `weight_provider_type`     |     `string`    |  String representing the type of the weight provider. The string should match the name of the C++ class. It is required only if the task is low priority. The default value in case of low priority task (`priority = 1`) is `ConstantWeightProvider`            |     No    |
     * Given the weight type specified by `weight_provider_type`, the user must specify all the
     * parameters required by the provider in the `TASK_NAME` group handler
     * `TASK_NAME` is a placeholder for the name of the task contained in the `tasks` list.
     * @note The following `ini` file presents an example of the configuration that can be used to
     * build the TSID
     * ~~~~~{.ini}
     * tasks   ("COM", "ROOT", "CHEST",
     *          "LEFT_FOOT", "RIGHT_FOOT",
     *          "LEFT_FOOT_WRENCH", "RIGHT_FOOT_WRENCH",
     *          "JOINT_REGULARIZATION", "JOINT_DYNAMICS_TASK", "BASE_DYNAMICS_TASK")
     *
     * [TSID]
     * robot_acceleration_variable_name  "robot_acceleration"
     * joint_torques_variable_name  "joint_torques"
     * contact_wrench_variables_name ("lf_wrench", "rf_wrench")
     *
     * [LEFT_FOOT]
     * type                              SE3Task
     * priority                          0
     * kp_linear                         500.0
     * kp_angular                        50.0
     * kd_linear                         14.0
     * kd_angular                        14.0
     * frame_name                        l_sole
     *
     * [RIGHT_FOOT]
     * type                              SE3Task
     * priority                          0
     * kp_linear                         500.0
     * kp_angular                        50.0
     * kd_linear                         14.0
     * kd_angular                        14.0
     * frame_name                        r_sole
     *
     * [COM]
     * type                              CoMTask
     * kp_linear                         20.0
     * kd_linear                         5.0
     * priority                          0
     * mask                              (true, true, false)
     *
     * [CHEST]
     * type                              SO3Task
     * kp_angular                        50.0
     * kd_angular                        4.47
     * frame_name                        chest
     * priority                          1
     * weight                            (10.0, 10.0, 10.0)
     *
     * [ROOT]
     * type                              R3Task
     * frame_name                        root_link
     * kp_linear                         5.0
     * kd_linear                         4.47
     * mask                              (false, false, true)
     * priority                          0
     *
     * [JOINT_REGULARIZATION]
     * type                              JointTrackingTask
     * priority                          1
     * kp                                (5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
     *                                    5.0, 5.0, 5.0, 5.0, 5.0, 5.0
     *                                    5.0, 5.0, 5.0,
     *                                    5.0, 5.0, 5.0,
     *                                    5.0, 5.0, 5.0, 5.0,
     *                                    5.0, 5.0, 5.0, 5.0)
     *
     * weight                           (1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
     *                                   1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
     *                                   1.0, 1.0, 1.0,
     *                                   1.0, 1.0, 1.0,
     *                                   2.0, 2.0, 2.0, 2.0,
     *                                   2.0, 2.0, 2.0, 2.0)
     *
     * [LEFT_FOOT_WRENCH]
     * type                             FeasibleContactWrenchTask
     * priority                         0
     * variable_name                    lf_wrench
     * frame_name                       l_sole
     * number_of_slices                 2
     * static_friction_coefficient      0.3
     * foot_limits_x                    (-0.1, 0.1)
     * foot_limits_y                    (-0.06, 0.06)
     *
     * [RIGHT_FOOT_WRENCH]
     * type                              FeasibleContactWrenchTask
     * priority                          0
     * variable_name                     rf_wrench
     * frame_name                        r_sole
     * number_of_slices                  2
     * static_friction_coefficient       0.3
     * foot_limits_x                    (-0.1, 0.1)
     * foot_limits_y                    (-0.06, 0.06)
     *
     * [include JOINT_DYNAMICS_TASK "./tasks/joint_dynamics_task.ini"]
     *
     * [include BASE_DYNAMICS_TASK "./tasks/base_dynamics_task.ini"]
     * ~~~~~
     * Where the file `./tasks/joint_dynamics_task.ini` and `./tasks/base_dynamics_task.ini`
     * contains the definition of the `JointsDynamicsTask` and `BaseDynamicsTask`.
     * Since the tasks requires the definition of subgroups, an additional file is
     * suggested as explained in: https://github.com/robotology/yarp/discussions/2563
     * The following code represent the content of the `./tasks/joint_dynamics_task.ini`, the other file
     * can be easily built checking the documentation
     * ~~~~~{.ini}
     * type                               JointDynamicsTask
     * priority                           0
     *
     * max_number_of_contacts             2
     *
     * [CONTACT_0]
     * variable_name                      lf_wrench
     * frame_name                         l_sole
     *
     * [CONTACT_1]
     * variable_name                      rf_wrench
     * frame_name                         r_sole
     * ~~~~~
     * @return an TaskSpaceInverseDynamicsProblem. In case of issues an invalid TaskSpaceInverseDynamicsProblem
     * will be returned.
     */
    static TaskSpaceInverseDynamicsProblem
    build(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
          std::shared_ptr<iDynTree::KinDynComputations> kinDyn);
    // clang-format on
};

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_QP_TSID_H
