/**
 * @file ILinearTaskSolver.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_ILINEAR_TASK_SOLVER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_ILINEAR_TASK_SOLVER_H

#include <memory>
#include <string>
#include <type_traits>

#include <Eigen/Dense>

#include <iDynTree/Core/MatrixView.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Source.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

namespace BipedalLocomotion
{
namespace System
{

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
     * @param priority Priority associated to the task. The lower the number the higher the
     * priority.
     * @param weight Weight associated to the task. This parameter is optional. The default value is
     * an object that does not contain any value. So is an invalid weight.
     * @return true if the task has been added to the inverse kinematics.
     */
    virtual bool addTask(std::shared_ptr<Task> task,
                         const std::string& taskName,
                         std::size_t priority,
                         std::optional<Eigen::Ref<const Eigen::VectorXd>> weight = {})
        = 0;

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
     * Destructor.
     */
    virtual ~ILinearTaskSolver() = default;
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_ILINEAR_TASK_SOLVER_H
