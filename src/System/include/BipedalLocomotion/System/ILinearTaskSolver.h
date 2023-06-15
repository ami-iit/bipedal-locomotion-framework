/**
 * @file ILinearTaskSolver.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_ILINEAR_TASK_SOLVER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_ILINEAR_TASK_SOLVER_H

#include <memory>
#include <optional>
#include <string>
#include <type_traits>

#include <Eigen/Dense>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/LinearTask.h>
#include <BipedalLocomotion/System/OutputPort.h>
#include <BipedalLocomotion/System/Source.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

namespace BipedalLocomotion
{
namespace System
{

/**
 * WeightProviderPort port representing the weight
 */
using WeightProviderPort = OutputPort<Eigen::VectorXd>;

/**
 * ILinearTaskSolver describes the interface for solving problem related to LinearTask class. Please
 * check IntegrationBasedIK for further details.
 */
template <class _Task, class _State> class ILinearTaskSolver : public Source<_State>
{
    static_assert(std::is_base_of_v<LinearTask, _Task>,
                  "The _Task template argument of ILinearTaskSolver must inherits from "
                  "LinearTask");

public:
    using Task = _Task;
    using State = _State;

    /**
     * Add a linear task in the solver.
     * @param task pointer to a given linear task
     * @param taskName unique name associated to the task.
     * @param priority Priority associated to the task. The lower the number the higher the
     * priority.
     * @param weightProvider Weight provider associated to the task. This parameter is optional. The
     * default value is an object that does not contain any value. So is an invalid provider.
     * @return true if the task has been added to the solver.
     */
    virtual bool addTask(std::shared_ptr<Task> task,
                         const std::string& taskName,
                         std::size_t priority,
                         std::shared_ptr<const System::WeightProviderPort> weightProvider = nullptr)
        = 0;

    /**
     * Add a linear task in the solver.
     * @param task pointer to a given linear task
     * @param taskName unique name associated to the task.
     * @param priority Priority associated to the task. The lower the number the higher the
     * priority.
     * @param weight Weight associated to the task.
     * @return true if the task has been added to the solver.
     * @note The solver assumes the weight is a constant value.
     */
    virtual bool addTask(std::shared_ptr<Task> task,
                         const std::string& taskName,
                         std::size_t priority,
                         Eigen::Ref<const Eigen::VectorXd> weight)
        = 0;

    /**
     * Set the weightProvider associated to an already existing task
     * @param taskName name associated to the task
     * @param weightProvider new weight provider associated to the task.
     * @return true if the weight has been updated
     */
    virtual bool setTaskWeight(const std::string& taskName,
                               std::shared_ptr<const WeightProviderPort> weightProvider)
        = 0;

    /**
     * Set the weight associated to an already existing task
     * @param taskName name associated to the task
     * @param weight new Weight associated to the task. A constant weight is assumed.
     * @return true if the weight has been updated
     */
    virtual bool setTaskWeight(const std::string& taskName,
                               Eigen::Ref<const Eigen::VectorXd> weight)
        = 0;

    /**
     * Get the weightProvider associated to an already existing task
     * @param taskName name associated to the task
     * @return a weak pointer to the weightProvider. If the task does not exist the pointer is not
     * lockable
     */
    virtual std::weak_ptr<const WeightProviderPort>
    getTaskWeightProvider(const std::string& taskName) const = 0;

    /**
     * Get a vector containing the name of the tasks.
     * @return an std::vector containing all the names associated to the tasks
     */
    virtual std::vector<std::string> getTaskNames() const = 0;

    /**
     * Finalize the Solver.
     * @param handler parameter handler.
     * @note You should call this method after you add ALL the tasks.
     * @return true in case of success, false otherwise.
     */
    virtual bool finalize(const System::VariablesHandler& handler) = 0;

    /**
     * Get a specific task
     * @param name name associated to the task.
     * @return a weak ptr associated to an existing task in the solver. If the task does not exist a
     * nullptr is returned.
     */
    virtual std::weak_ptr<Task> getTask(const std::string& name) const = 0;

    /**
     * Return the description of the solver problem.
     * @return a string containing the description of the solver.
     */
    virtual std::string toString() const = 0;

    /**
     * Return the vector representing the entire solution of the ILinearTaskSolver.
     * @return a vector containing the solution of the solver
     */
    virtual Eigen::Ref<const Eigen::VectorXd> getRawSolution() const = 0;

    /**
     * Destructor.
     */
    virtual ~ILinearTaskSolver() = default;
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_ILINEAR_TASK_SOLVER_H
