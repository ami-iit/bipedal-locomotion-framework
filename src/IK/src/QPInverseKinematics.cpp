/**
 * @file QPInverseKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <cstddef>
#include <memory>

#include <iDynTree/KinDynComputations.h>

#include <OsqpEigen/OsqpEigen.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/System/ConstantWeightProvider.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/System/WeightProvider.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::IK;
using namespace BipedalLocomotion;

struct QPInverseKinematics::Impl
{
    struct TaskWithPriority
    {
        std::shared_ptr<QPInverseKinematics::Task> task;
        std::size_t priority;
        std::shared_ptr<const System::WeightProviderPort> weightProvider;
        Eigen::MatrixXd tmp; /**< This is a temporary matrix usefull to reduce dynamics allocation
                                in advance method */
    };

    QPInverseKinematics::State solution;

    System::VariablesHandler::VariableDescription robotVelocityVariable;

    OsqpEigen::Solver solver; /**< Optimization solver. */
    bool isVerbose{false};

    std::unordered_map<std::string, TaskWithPriority> tasks;

    std::vector<std::reference_wrapper<const TaskWithPriority>> constraints;
    std::vector<std::reference_wrapper<TaskWithPriority>> costs;
    std::size_t numberOfConstraints;

    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient;
    Eigen::MatrixXd constraintMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    bool isFirstIteration{true};
    bool isValid{false};
    bool isInitialized{false};
    bool isFinalized{false};

    bool initializeSolver()
    {
        constexpr auto logPrefix = "[QPInversekinematics::Impl::initializeSolver]";
        // Hessian matrix
        Eigen::SparseMatrix<double> hessianSparse = this->hessian.sparseView();
        if (!this->solver.data()->setHessianMatrix(hessianSparse))
        {
            log()->error("{} Unable to set the hessian matrix.", logPrefix);
            return false;
        }

        // gradient
        if (!this->solver.data()->setGradient(gradient))
        {
            log()->error("{} Unable to set the gradient vector.", logPrefix);
            return false;
        }

        Eigen::SparseMatrix<double> constraintsMatrixSparse = this->constraintMatrix.sparseView();
        if (!this->solver.data()->setLinearConstraintsMatrix(constraintsMatrixSparse))
        {
            log()->error("{} Unable to set the constraint matrix.", logPrefix);
            return false;
        }

        if (!this->solver.data()->setBounds(this->lowerBound, this->upperBound))
        {
            log()->error("{} Unable to set the bounds.", logPrefix);
            return false;
        }

        if (!this->solver.initSolver())
        {
            log()->error("{} Unable to initialize the solver.", logPrefix);
            return false;
        }

        return true;
    }

    bool updateSolver()
    {
        constexpr auto logPrefix = "[QPInversekinematics::Impl::updateSolver]";
        // Hessian matrix
        Eigen::SparseMatrix<double> hessianSparse = this->hessian.sparseView();
        if (!this->solver.updateHessianMatrix(hessianSparse))
        {
            log()->error("{} Unable to set the hessian matrix.", logPrefix);
            return false;
        }

        // gradient
        if (!this->solver.updateGradient(gradient))
        {
            log()->error("{} Unable to set the gradient vector.", logPrefix);
            return false;
        }

        Eigen::SparseMatrix<double> constraintsMatrixSparse = this->constraintMatrix.sparseView();
        if (!this->solver.updateLinearConstraintsMatrix(constraintsMatrixSparse))
        {
            log()->error("{} Unable to set the constraint matrix.", logPrefix);
            return false;
        }

        if (!this->solver.updateBounds(this->lowerBound, this->upperBound))
        {
            log()->error("{} Unable to set the bounds.", logPrefix);
            return false;
        }

        return true;
    }
};

QPInverseKinematics::QPInverseKinematics()
{
    m_pimpl = std::make_unique<QPInverseKinematics::Impl>();
}

QPInverseKinematics::~QPInverseKinematics() = default;

bool QPInverseKinematics::addTask(std::shared_ptr<QPInverseKinematics::Task> task,
                                  const std::string& taskName,
                                  std::size_t priority,
                                  Eigen::Ref<const Eigen::VectorXd> weight)
{
    return this->addTask(task,
                         taskName,
                         priority,
                         std::make_shared<System::ConstantWeightProvider>(weight));
}

bool QPInverseKinematics::addTask(std::shared_ptr<QPInverseKinematics::Task> task,
                                  const std::string& taskName,
                                  std::size_t priority,
                                  std::shared_ptr<const System::WeightProviderPort> weightProvider)
{
    constexpr auto logPrefix = "[QPInverseKinematics::addTask]";

    // check if the task already exist
    const bool taskExist = (m_pimpl->tasks.find(taskName) != m_pimpl->tasks.end());
    if (taskExist)
    {
        log()->error("{} The task named {} already exist.", logPrefix, taskName);
        return false;
    }

    if (priority != 0 && priority != 1)
    {
        log()->error("{} - [Task name: '{}'] For the time being we support only priority equal to "
                     "0 or 1.",
                     logPrefix,
                     taskName);
        return false;
    }

    if (priority == 1 && task->type() == System::LinearTask::Type::inequality)
    {
        log()->error("{} - [Task name: '{}'] This implementation of the inverse kinematics cannot "
                     "handle inequality tasks with priority equal to 1.",
                     logPrefix,
                     taskName);
        return false;
    }

    // Store the task inside the InverseKinematics
    m_pimpl->tasks[taskName].task = task;
    m_pimpl->tasks[taskName].priority = priority;

    // If the priority is set to 1 the user has to provide the weight in terms of weight provider
    if (priority == 1 && !weightProvider)
    {
        log()->error("{} - [Task name: '{}'] Please provide the associated weight. This is "
                     "necessary since the priority of the task is equal to 1",
                     logPrefix,
                     taskName);

        // erase the task since it is not valid
        m_pimpl->tasks.erase(taskName);

        return false;
    }

    if (priority == 1 && weightProvider)
    {
        if (weightProvider->getOutput().size() != task->size())
        {
            log()->error("{} - [Task name: '{}'] The size of the weight is not coherent with the "
                         "size of the task. Expected: {}. Given: {}.",
                         logPrefix,
                         taskName,
                         m_pimpl->tasks[taskName].task->size(),
                         weightProvider->getOutput().size());

            // erase the task since it is not valid
            m_pimpl->tasks.erase(taskName);

            return false;
        }

        // add the weight In this case we assume that the weight will be constant
        m_pimpl->tasks[taskName].weightProvider = weightProvider;

        // add the task to the list of the element that are used to build the cost function
        m_pimpl->costs.push_back(m_pimpl->tasks[taskName]);
    } else
    {
        m_pimpl->constraints.push_back(m_pimpl->tasks[taskName]);
    }

    // if you add a new task you should call finalize again
    m_pimpl->isFinalized = false;

    return true;
}

bool QPInverseKinematics::setTaskWeight(
    const std::string& taskName, std::shared_ptr<const System::WeightProviderPort> weightProvider)
{
    constexpr auto logPrefix = "[QPInverseKinematics::setTaskWeight]";

    auto tmp = m_pimpl->tasks.find(taskName);

    const bool taskExist = (tmp != m_pimpl->tasks.end());
    if (!taskExist)
    {
        log()->error("{} The task named {} does not exist.", logPrefix, taskName);
        return false;
    }

    auto taskWithPriority = tmp->second;

    if (taskWithPriority.priority != 1)
    {
        log()->error("{} - [Task name: '{}'] The weight can be set only to a task with priority "
                     "equal to 1.",
                     logPrefix,
                     taskName);
        return false;
    }

    if (weightProvider == nullptr || !weightProvider->isOutputValid())
    {
        log()->error("{} - [Task name: '{}'] The weightProvider is not valid.",
                     logPrefix,
                     taskName);
        return false;
    }

    if (weightProvider->getOutput().size() != taskWithPriority.task->size())
    {
        log()->error("{} - [Task name: '{}'] The size of the weight is not coherent with the "
                     "size of the task. Expected: {}. Given: {}.",
                     logPrefix,
                     taskName,
                     taskWithPriority.task->size(),
                     weightProvider->getOutput().size());
        return false;
    }

    // update the weight
    m_pimpl->tasks[taskName].weightProvider = weightProvider;

    return true;
}

bool QPInverseKinematics::setTaskWeight(const std::string& taskName,
                                        Eigen::Ref<const Eigen::VectorXd> weight)
{
    return this->setTaskWeight(taskName, std::make_shared<System::ConstantWeightProvider>(weight));
}

std::weak_ptr<const System::WeightProviderPort>
QPInverseKinematics::getTaskWeightProvider(const std::string& taskName) const
{
    constexpr auto logPrefix = "[QPInverseKinematics::getTaskWeightProvider]";

    auto taskWithPriority = m_pimpl->tasks.find(taskName);
    const bool taskExist = (taskWithPriority != m_pimpl->tasks.end());
    if (!taskExist)
    {
        log()->error("{} The task named {} does not exist.", logPrefix, taskName);
        return std::shared_ptr<System::WeightProviderPort>();
    }

    return taskWithPriority->second.weightProvider;
}

bool QPInverseKinematics::finalize(const System::VariablesHandler& handler)
{
    constexpr auto logPrefix = "[QPInverseKinematics::finalize]";

    if (!m_pimpl->isInitialized)
    {
        log()->error("{} Please call initialize() before finalize().", logPrefix);
        return false;
    }

    m_pimpl->isFinalized = false;

    // clear the solver
    m_pimpl->solver.clearSolver();

    // set the some internal parameter of osqp-eigen
    m_pimpl->solver.settings()->setVerbosity(m_pimpl->isVerbose);

    // set the variable handler for all the tasks
    m_pimpl->numberOfConstraints = 0;
    for (auto& [name, task] : m_pimpl->tasks)
    {
        if (!task.task->setVariablesHandler(handler))
        {
            log()->error("{} Error while setting the variable handler in the task named {}, having "
                         "the following description {}.",
                         logPrefix,
                         name,
                         task.task->getDescription());
            return false;
        }

        // if priority is equal to 0 the task is considered as hard constraint.
        if (task.priority == 0)
        {
            m_pimpl->numberOfConstraints += task.task->size();
        }
    }

    // resize the temporary matrix usefull to reduce dynamics allocation when advance() is called
    for (auto& cost : m_pimpl->costs)
    {
        if(cost.get().weightProvider == nullptr)
        {
            log()->error("{} One of the weight provider has been not correctly set.",
                         logPrefix);
            return false;
        }

        cost.get().tmp.resize(handler.getNumberOfVariables(),
                              cost.get().weightProvider->getOutput().size());
    }

    m_pimpl->solver.data()->setNumberOfVariables(handler.getNumberOfVariables());
    m_pimpl->solver.data()->setNumberOfConstraints(m_pimpl->numberOfConstraints);

    // resize matrices
    m_pimpl->hessian.resize(handler.getNumberOfVariables(), handler.getNumberOfVariables());
    m_pimpl->gradient.resize(handler.getNumberOfVariables());

    m_pimpl->constraintMatrix.resize(m_pimpl->numberOfConstraints, handler.getNumberOfVariables());
    m_pimpl->upperBound.resize(m_pimpl->numberOfConstraints);
    m_pimpl->lowerBound.resize(m_pimpl->numberOfConstraints);

    if (!handler.getVariable(m_pimpl->robotVelocityVariable.name, m_pimpl->robotVelocityVariable))
    {
        log()->error("{} Error while retrieving the robot velocity variable.", logPrefix);
        return false;
    }

    m_pimpl->isFinalized = true;

    return true;
}

std::vector<std::string> QPInverseKinematics::getTaskNames() const
{
    std::vector<std::string> tasksName;

    for (const auto& [name, task] : m_pimpl->tasks)
    {
        tasksName.push_back(name);
    }

    return tasksName;
}

bool QPInverseKinematics::advance()
{
    constexpr auto logPrefix = "[QPInverseKinematics::advance]";

    // when advance is called the previous solution is no more valid
    m_pimpl->isValid = false;

    if (!m_pimpl->isInitialized)
    {
        log()->error("{} Please call initialize() before advance().", logPrefix);
        return false;
    }

    if (!m_pimpl->isFinalized)
    {
        log()->error("{} Please call finalize() before advance().", logPrefix);
        return false;
    }


    // update of all the tasks
    for (auto& [name, task] : m_pimpl->tasks)
    {
        if (!task.task->update())
        {
            log()->error("{} Unable to update the task named {}.", logPrefix, name);
            return false;
        }

        // the outcome of isValid() should be the same of update. This test is required
        assert(task.task->isValid() && "One of the task is not valid.");
    }

    // Compute the gradient and the hessian
    m_pimpl->hessian.setZero();
    m_pimpl->gradient.setZero();
    for (auto& cost : m_pimpl->costs)
    {
        Eigen::Ref<const Eigen::MatrixXd> A = cost.get().task->getA();
        Eigen::Ref<const Eigen::VectorXd> b = cost.get().task->getB();

        // Here we avoid to have dynamic allocation
        cost.get().tmp.noalias() = A.transpose() * //
                                   cost.get().weightProvider->getOutput().asDiagonal();
        m_pimpl->hessian.noalias() += cost.get().tmp * A;
        m_pimpl->gradient.noalias() -= cost.get().tmp * b;
    }

    // compute the constraints
    std::size_t index = 0;
    for (const auto& constraint : m_pimpl->constraints)
    {
        // check if the number of constraints changed
        if (m_pimpl->numberOfConstraints < index + constraint.get().task->size())
        {
            log()->error("{} The number of constraints changed. Please call finalize() again. "
                         "Expected number of constraints: {}. Adding the constrained named {}, the "
                         "number of constraints becomes {}.",
                         logPrefix,
                         m_pimpl->numberOfConstraints,
                         constraint.get().task->getDescription(),
                         index + constraint.get().task->size());
            return false;
        }

        Eigen::Ref<const Eigen::MatrixXd> A = constraint.get().task->getA();
        Eigen::Ref<const Eigen::VectorXd> b = constraint.get().task->getB();

        m_pimpl->constraintMatrix.middleRows(index, constraint.get().task->size()) = A;
        m_pimpl->upperBound.segment(index, constraint.get().task->size()) = b;

        if (constraint.get().task->type() == System::LinearTask::Type::inequality)
        {
            m_pimpl->lowerBound.segment(index, constraint.get().task->size())
                .setConstant(-OsqpEigen::INFTY);
        } else
        {
            assert(constraint.get().task->type() == System::LinearTask::Type::equality);

            m_pimpl->lowerBound.segment(index, constraint.get().task->size()) = b;
        }

        index += constraint.get().task->size();
    }

    // update the solver
    if (!m_pimpl->isFirstIteration)
    {
        if (!m_pimpl->updateSolver())
        {
            log()->error("{} Unable to update the QP solver.", logPrefix);
            return false;
        }
    } else
    {
        if (!m_pimpl->initializeSolver())
        {
            log()->error("{} Unable to run the QP solver the first time.", logPrefix);
            return false;
        }
        m_pimpl->isFirstIteration = false;
    }

    // solve the QP
    if (m_pimpl->solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
        log()->error("{} Unable to to solve the problem.", logPrefix);
        return false;
    }

    if (m_pimpl->solver.getStatus() != OsqpEigen::Status::Solved
        && m_pimpl->solver.getStatus() != OsqpEigen::Status::SolvedInaccurate)
    {
        log()->error("{} osqp was not able to find a feasible solution.", logPrefix);
        return false;
    }

    if (m_pimpl->solver.getStatus() == OsqpEigen::Status::SolvedInaccurate)
    {
        log()->debug("{} The solver found an inaccurate feasible solution.", logPrefix);
    }

    // retrieve the solution
    constexpr std::size_t spatialVelocitySize = 6;
    const std::size_t joints = m_pimpl->robotVelocityVariable.size - spatialVelocitySize;

    m_pimpl->solution.jointVelocity
        = m_pimpl->solver.getSolution().segment(m_pimpl->robotVelocityVariable.offset
                                                    + spatialVelocitySize,
                                                joints);

    m_pimpl->solution.baseVelocity.coeffs()
        = m_pimpl->solver.getSolution().segment<spatialVelocitySize>(
            m_pimpl->robotVelocityVariable.offset);

    m_pimpl->isValid = true;

    return true;
}

bool QPInverseKinematics::isOutputValid() const
{
    return m_pimpl->isValid;
}

const QPInverseKinematics::State& QPInverseKinematics::getOutput() const
{
    return m_pimpl->solution;
}

std::weak_ptr<QPInverseKinematics::Task> QPInverseKinematics::getTask(const std::string& name) const
{
    constexpr auto logPrefix = "[QPInverseKinematics::getTask]";
    auto task = m_pimpl->tasks.find(name);

    if (task == m_pimpl->tasks.end())
    {
        log()->warn("{} The task named {} does not exist. A nullptr will be returned.",
                    logPrefix,
                    name);
        return std::shared_ptr<QPInverseKinematics::Task>(nullptr);
    }

    return task->second.task;
}

bool QPInverseKinematics::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[QPInverseKinematics::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("robot_velocity_variable_name", m_pimpl->robotVelocityVariable.name))
    {
        log()->error("{} Error while retrieving the robot velocity variable.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("verbosity", m_pimpl->isVerbose))
    {
        log()->info("{} 'verbosity' not found. The following parameter will be used '{}'.",
                    logPrefix,
                    m_pimpl->isVerbose);
    }

    m_pimpl->isInitialized = true;

    return true;
}

std::string QPInverseKinematics::toString() const
{
    std::ostringstream oss;

    oss << "====== QPInverseKinematics class ======" << std::endl
        << "The optimization problem is composed by the following tasks:" << std::endl;
    for (const auto& [name, task] : m_pimpl->tasks)
    {
        oss << "\t - " << name << ": " << task.task->getDescription()
            << " Priority: " << task.priority << "." << std::endl;
    }
    oss << "Note: The lower is the integer associated to the priority, the higher is the priority."
        << std::endl
        << "==========================" << std::endl;

    return oss.str();
}

Eigen::Ref<const Eigen::VectorXd> QPInverseKinematics::getRawSolution() const
{
    return m_pimpl->solver.getSolution();
}

std::pair<BipedalLocomotion::System::VariablesHandler, std::unique_ptr<QPInverseKinematics>>
QPInverseKinematics::build(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                           std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    constexpr auto logPrefix = "[QPInverseKinematics::build]";
    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return std::make_pair(BipedalLocomotion::System::VariablesHandler(), nullptr);
    }

    std::unique_ptr<QPInverseKinematics> solver = std::make_unique<QPInverseKinematics>();

    if (!solver->initialize(ptr->getGroup("IK")))
    {
        log()->error("{} Unable to initialize the IK solver.", logPrefix);
        return std::make_pair(BipedalLocomotion::System::VariablesHandler(), nullptr);
    }

    BipedalLocomotion::System::VariablesHandler variableHandler;
    if (!variableHandler.addVariable(solver->m_pimpl->robotVelocityVariable.name,
                                     kinDyn->getNrOfDegreesOfFreedom() + 6))
    {
        log()->error("{} Unable to add the variable named '{}'.",
                     logPrefix,
                     solver->m_pimpl->robotVelocityVariable.name);
        return std::make_pair(BipedalLocomotion::System::VariablesHandler(), nullptr);
    }

    std::vector<std::string> tasks;
    if (!ptr->getParameter("tasks", tasks))
    {
        log()->error("{} Unable to find the parameter 'tasks'.", logPrefix);
        return std::make_pair(BipedalLocomotion::System::VariablesHandler(), nullptr);
    }

    for (const auto& taskGroupName : tasks)
    {
        auto taskGroupTmp = ptr->getGroup(taskGroupName).lock();
        if (taskGroupTmp == nullptr)
        {
            log()->error("{} Unable to find the group named '{}'.", logPrefix, taskGroupName);
            return std::make_pair(BipedalLocomotion::System::VariablesHandler(), nullptr);
        }

        // add robot_velocity_variable_name parameter since it is required by all the tasks
        auto taskGroup = taskGroupTmp->clone();
        taskGroup->setParameter("robot_velocity_variable_name",
                                solver->m_pimpl->robotVelocityVariable.name);

        // create an instance of the task
        std::string taskType;
        if (!taskGroup->getParameter("type", taskType))
        {
            log()->error("{} Unable to find the parameter 'type' in the group named '{}'.",
                         logPrefix,
                         taskGroupName);
            return std::make_pair(BipedalLocomotion::System::VariablesHandler(), nullptr);
        }

        std::shared_ptr<IKLinearTask> taskInstance = IKLinearTaskFactory::createInstance(taskType);
        if (taskInstance == nullptr)
        {
            log()->error("{} The task type '{}' has not been registered.", logPrefix, taskType);
            return std::make_pair(BipedalLocomotion::System::VariablesHandler(), nullptr);
        }

        if (!taskInstance->setKinDyn(kinDyn))
        {
            log()->error("{} Unable to set the kinDynComputations object for the task in the group "
                         "'{}'.",
                         logPrefix,
                         taskGroupName);
            return std::make_pair(BipedalLocomotion::System::VariablesHandler(), nullptr);
        }

        int priority{0};
        if (!taskGroup->getParameter("priority", priority))
        {
            log()->error("{} Unable to get the parameter 'priority' for the task in the group "
                         "'{}'.",
                         logPrefix,
                         taskGroupName);
            return std::make_pair(BipedalLocomotion::System::VariablesHandler(), nullptr);
        }

        Eigen::VectorXd weight;
        if (priority == 1)
        {
            if (!taskGroup->getParameter("weight", weight))
            {
                log()->error("{} Unable to get the parameter 'weight' for the task in the group "
                             "'{}'.",
                             logPrefix,
                             taskGroupName);
                return std::make_pair(BipedalLocomotion::System::VariablesHandler(), nullptr);
            }
        }

        if (!taskInstance->initialize(taskGroup))
        {
            log()->error("{} Unable to task named '{}'.", logPrefix, taskGroupName);
            return std::make_pair(BipedalLocomotion::System::VariablesHandler(), nullptr);
        }

        // the weight is considered only if priority is equal to 1
        if (!solver->addTask(taskInstance, taskGroupName, priority, weight))
        {
            log()->error("{} Unable to add the task named '{}' in the solver.",
                         logPrefix,
                         taskGroupName);
            return std::make_pair(BipedalLocomotion::System::VariablesHandler(), nullptr);
        }
    }

    if (!solver->finalize(variableHandler))
    {
        log()->error("{} Unable to finalize the solver.", logPrefix);
        return std::make_pair(BipedalLocomotion::System::VariablesHandler(), nullptr);
    }

    return std::make_pair(std::move(variableHandler), std::move(solver));
}
