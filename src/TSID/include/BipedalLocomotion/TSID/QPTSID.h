/**
 * @file QPTSID.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TSID_QP_TSID_H
#define BIPEDAL_LOCOMOTION_TSID_QP_TSID_H

#include <memory>
#include <optional>
#include <functional>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/TSID/TaskSpaceInverseDynamics.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/System/WeightProvider.h>

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
};

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_QP_TSID_H
