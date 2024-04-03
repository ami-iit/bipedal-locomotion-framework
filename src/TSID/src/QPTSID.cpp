/**
 * @file QPTSID.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <OsqpEigen/Constants.hpp>
#include <OsqpEigen/OsqpEigen.h>

#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/System/ConstantWeightProvider.h>
#include <BipedalLocomotion/System/ILinearTaskSolver.h>
#include <BipedalLocomotion/TSID/QPTSID.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::TSID;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;

struct QPTSID::Impl
{
    struct TaskWithPriority
    {
        std::shared_ptr<QPTSID::Task> task;
        std::size_t priority;
        std::shared_ptr<const System::WeightProviderPort> weightProvider;
        Eigen::MatrixXd tmp; /**< This is a temporary matrix useful to reduce dynamics allocation
                                in advance method */
    };

    std::unordered_map<std::string, TaskWithPriority> tasks; /**< This is the task list containg the
                                                                tasks defined both internally to the
                                                                class and externally by the user.
                                                                The tasks are both constraints and
                                                                costs. */
    std::vector<std::reference_wrapper<const TaskWithPriority>> constraints; /**< List of tasks
                                                                                representing
                                                                                constraints
                                                                                (priority = 0). */
    std::vector<std::reference_wrapper<TaskWithPriority>> costs; /**< List of tasks representing
                                                                    costs (priority = 1). */
    std::size_t numberOfConstraints;

    bool isVerbose{false};

    bool isFirstIteration{true};
    bool isValid{false};
    bool isInitialized{false};
    bool isFinalized{false};

    QPTSID::State solution;

    VariablesHandler::VariableDescription robotAccelerationVariable;
    VariablesHandler::VariableDescription jointTorquesVariable;

    std::vector<VariablesHandler::VariableDescription> contactWrenchVariables;

    OsqpEigen::Solver solver; /**< Optimization solver. */

    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient;
    Eigen::MatrixXd constraintMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    bool initializeSolver()
    {
        constexpr auto logPrefix = "[QPTSID::Impl::initializeSolver]";

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

        // if the number of constraints is equal to zero we do not need to set the constraint
        if (this->numberOfConstraints > 0)
        {
            Eigen::SparseMatrix<double> constraintsMatrixSparse
                = this->constraintMatrix.sparseView();
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
        constexpr auto logPrefix = "[QPTSID::Impl::updateSolver]";
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

        // if the number of constraints is equal to zero we do not need to update the constraint
        if (this->numberOfConstraints == 0)
        {
            return true;
        }

        // In this case the number of constraints is not equal to zero. We need to update the
        // constraint matrix and the bounds.
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

QPTSID::QPTSID()
{
    m_pimpl = std::make_unique<QPTSID::Impl>();
}

QPTSID::~QPTSID() = default;

bool QPTSID::addTask(std::shared_ptr<QPTSID::Task> task,
                     const std::string& taskName,
                     std::size_t priority,
                     Eigen::Ref<const Eigen::VectorXd> weight)
{
    return this->addTask(task,
                         taskName,
                         priority,
                         std::make_shared<System::ConstantWeightProvider>(weight));
}

bool QPTSID::addTask(std::shared_ptr<QPTSID::Task> task,
                     const std::string& taskName,
                     std::size_t priority,
                     std::shared_ptr<const System::WeightProviderPort> weightProvider)
{
    constexpr auto logPrefix = "[QPTSID::addTask]";

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

    if (priority == 1 && task->type() == QPTSID::Task::Type::inequality)
    {
        log()->error("{} - [Task name: '{}'] This implementation of the task space inverse "
                     "dynamics cannot "
                     "handle inequality tasks with priority equal to 1.",
                     logPrefix,
                     taskName);
        return false;
    }

    // Store the task inside the inverse dynamics
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

bool QPTSID::setTaskWeight(const std::string& taskName,
                           std::shared_ptr<const System::WeightProviderPort> weightProvider)
{
    constexpr auto logPrefix = "[QPTSID::setTaskWeight]";

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

bool QPTSID::setTaskWeight(const std::string& taskName, Eigen::Ref<const Eigen::VectorXd> weight)
{
    return this->setTaskWeight(taskName, std::make_shared<System::ConstantWeightProvider>(weight));
}

std::weak_ptr<const System::WeightProviderPort>
QPTSID::getTaskWeightProvider(const std::string& taskName) const
{
    constexpr auto logPrefix = "[QPTSID::getTaskWeightProvider]";

    auto taskWithPriority = m_pimpl->tasks.find(taskName);
    const bool taskExist = (taskWithPriority != m_pimpl->tasks.end());
    if (!taskExist)
    {
        log()->error("{} The task named {} does not exist.", logPrefix, taskName);
        return std::shared_ptr<System::WeightProviderPort>();
    }

    return taskWithPriority->second.weightProvider;
}

std::vector<std::string> QPTSID::getTaskNames() const
{
    std::vector<std::string> tasksName;

    for (const auto& [name, task] : m_pimpl->tasks)
    {
        tasksName.push_back(name);
    }

    return tasksName;
}

std::weak_ptr<QPTSID::Task> QPTSID::getTask(const std::string& name) const
{
    constexpr auto logPrefix = "[QPTSID::getTask]";
    auto task = m_pimpl->tasks.find(name);

    if (task == m_pimpl->tasks.end())
    {
        log()->warn("{} The task named {} does not exist. A nullptr will be returned.",
                    logPrefix,
                    name);
        return std::shared_ptr<QPTSID::Task>(nullptr);
    }

    return task->second.task;
}

bool QPTSID::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[QPTSID::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("verbosity", m_pimpl->isVerbose))
    {
        log()->info("{} 'verbosity' not found. The following parameter will be used '{}'.",
                    logPrefix,
                    m_pimpl->isVerbose);
    }

    if (!ptr->getParameter("robot_acceleration_variable_name",
                           m_pimpl->robotAccelerationVariable.name))
    {
        log()->error("{} Error while retrieving the robot acceleration variable.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("joint_torques_variable_name", m_pimpl->jointTorquesVariable.name))
    {
        log()->error("{} Error while retrieving the joint torques variable.", logPrefix);
        return false;
    }

    // get the contact wrench variables name
    std::vector<std::string> wrenchVariableNames;
    if (!ptr->getParameter("contact_wrench_variables_name", wrenchVariableNames))
    {
        log()->error("{} Error while retrieving the contact wrench variables.", logPrefix);
        return false;
    }

    VariablesHandler::VariableDescription tmp;
    for (const auto& name : wrenchVariableNames)
    {
        tmp.name = name;
        m_pimpl->contactWrenchVariables.push_back(tmp);
    }

    m_pimpl->isInitialized = true;

    return true;
}

bool QPTSID::finalize(const System::VariablesHandler& handler)
{
    constexpr auto logPrefix = "[QPTSID::finalize]";

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

    // resize the temporary matrix useful to reduce dynamics allocation when advance() is called
    for (auto& cost : m_pimpl->costs)
    {
        if (cost.get().weightProvider == nullptr)
        {
            log()->error("{} One of the weight provider has been not correctly set.", logPrefix);
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

    if (!handler.getVariable(m_pimpl->robotAccelerationVariable.name,
                             m_pimpl->robotAccelerationVariable))
    {
        log()->error("{} Error while retrieving the robot acceleration variable.", logPrefix);
        return false;
    }

    if (!handler.getVariable(m_pimpl->jointTorquesVariable.name, m_pimpl->jointTorquesVariable))
    {
        log()->error("{} Error while retrieving the joint torques variable.", logPrefix);
        return false;
    }

    for (auto& variable : m_pimpl->contactWrenchVariables)
    {
        if (!handler.getVariable(variable.name, variable))
        {
            log()->error("{} Error while retrieving the contact wrench variable named: {}.",
                         logPrefix,
                         variable.name);
            return false;
        }

        constexpr int wrenchSize = BipedalLocomotion::Math::Wrenchd::SizeAtCompileTime;
        if (variable.size != wrenchSize)
        {
            log()->error("{} The size of the variable named: {} is different from the expected "
                         "one. Expected: {}, retrieved: {}.",
                         logPrefix,
                         variable.name,
                         variable.size,
                         wrenchSize);
            return false;
        }
    }

    m_pimpl->isFinalized = true;

    return true;
}

bool QPTSID::advance()
{
    constexpr auto logPrefix = "[QPTSID::advance]";

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

        if (constraint.get().task->type() == QPTSID::Task::Type::inequality)
        {
            m_pimpl->lowerBound.segment(index, constraint.get().task->size())
                .setConstant(-OsqpEigen::INFTY);
        } else
        {
            assert(constraint.get().task->type() == QPTSID::Task::Type::equality);

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
    constexpr std::size_t spatialAccelerationSize = 6;
    const std::size_t joints = m_pimpl->robotAccelerationVariable.size - spatialAccelerationSize;

    // the first six elements are the base acceleration
    m_pimpl->solution.baseAcceleration
        = m_pimpl->solver.getSolution().segment<spatialAccelerationSize>(
            m_pimpl->robotAccelerationVariable.offset);

    m_pimpl->solution.jointAccelerations
        = m_pimpl->solver.getSolution().segment(m_pimpl->robotAccelerationVariable.offset
                                                    + spatialAccelerationSize,
                                                joints);

    m_pimpl->solution.jointTorques
        = m_pimpl->solver.getSolution().segment(m_pimpl->jointTorquesVariable.offset,
                                                m_pimpl->jointTorquesVariable.size);

    for (const auto& variable : m_pimpl->contactWrenchVariables)
    {
        m_pimpl->solution.contactWrenches[variable.name].wrench
            = m_pimpl->solver.getSolution().segment(variable.offset, variable.size);
    }

    m_pimpl->isValid = true;

    return true;
}

const QPTSID::State& QPTSID::getOutput() const
{
    return m_pimpl->solution;
}

bool QPTSID::isOutputValid() const
{
    return m_pimpl->isValid;
}

std::string QPTSID::toString() const
{
    std::ostringstream oss;

    oss << "====== QPTSID class ======" << std::endl
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

Eigen::Ref<const Eigen::VectorXd> QPTSID::getRawSolution() const
{
    return m_pimpl->solver.getSolution();
}

TaskSpaceInverseDynamicsProblem
QPTSID::build(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
              std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    constexpr auto logPrefix = "[QPTSID::build]";
    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return TaskSpaceInverseDynamicsProblem();
    }

    if (kinDyn == nullptr || !kinDyn->isValid())
    {
        log()->error("{} Invalid iDynTree::KinDynComputations object.", logPrefix);
        return TaskSpaceInverseDynamicsProblem();
    }

    std::unique_ptr<QPTSID> solver = std::make_unique<QPTSID>();
    std::unordered_map<std::string, std::shared_ptr<System::WeightProvider>> weights;

    if (!solver->initialize(ptr->getGroup("TSID")))
    {
        log()->error("{} Unable to initialize the TSID solver. Looking for a parameter group named "
                     "TSID.",
                     logPrefix);
        return TaskSpaceInverseDynamicsProblem();
    }

    BipedalLocomotion::System::VariablesHandler variablesHandler;
    variablesHandler.addVariable(solver->m_pimpl->robotAccelerationVariable.name,
                                 kinDyn->getNrOfDegreesOfFreedom() + 6);
    variablesHandler.addVariable(solver->m_pimpl->jointTorquesVariable.name,
                                 kinDyn->getNrOfDegreesOfFreedom());
    for (const auto& variable : solver->m_pimpl->contactWrenchVariables)
    {
        variablesHandler.addVariable(variable.name, 6);
    }

    std::vector<std::string> tasks;
    if (!ptr->getParameter("tasks", tasks))
    {
        log()->error("{} Unable to find the parameter 'tasks'.", logPrefix);
        return TaskSpaceInverseDynamicsProblem();
    }

    for (const auto& taskGroupName : tasks)
    {
        auto taskGroupTmp = ptr->getGroup(taskGroupName).lock();
        if (taskGroupTmp == nullptr)
        {
            log()->error("{} Unable to find the group named '{}'.", logPrefix, taskGroupName);
            return TaskSpaceInverseDynamicsProblem();
        }

        // add robot_velocity_variable_name parameter since it is required by all the tasks
        auto taskGroup = taskGroupTmp->clone();
        taskGroup->setParameter("robot_acceleration_variable_name",
                                solver->m_pimpl->robotAccelerationVariable.name);
        taskGroup->setParameter("joint_torques_variable_name",
                                solver->m_pimpl->jointTorquesVariable.name);

        // create an instance of the task
        std::string taskType;
        if (!taskGroup->getParameter("type", taskType))
        {
            log()->error("{} Unable to find the parameter 'type' in the group named '{}'.",
                         logPrefix,
                         taskGroupName);
            return TaskSpaceInverseDynamicsProblem();
        }

        auto taskInstance = TSIDLinearTaskFactory::createInstance(taskType);
        if (taskInstance == nullptr)
        {
            log()->error("{} The task type '{}' has not been registered.", logPrefix, taskType);
            return TaskSpaceInverseDynamicsProblem();
        }

        if (!taskInstance->setKinDyn(kinDyn))
        {
            log()->error("{} Unable to set the kinDynComputations object for the task in the group "
                         "'{}'.",
                         logPrefix,
                         taskGroupName);
            return TaskSpaceInverseDynamicsProblem();
        }

        if (!taskInstance->initialize(taskGroup))
        {
            log()->error("{} Unable to task named '{}'.", logPrefix, taskGroupName);
            return TaskSpaceInverseDynamicsProblem();
        }

        int priority{0};
        if (!taskGroup->getParameter("priority", priority))
        {
            log()->error("{} Unable to get the parameter 'priority' for the task in the group "
                         "'{}'.",
                         logPrefix,
                         taskGroupName);
            return TaskSpaceInverseDynamicsProblem();
        }

        if (priority != 0 && priority != 1)
        {
            log()->error("{} Invalid priority provided for the task named '{}'. For the time being "
                         "we support only priority equal to 0 or 1.",
                         logPrefix,
                         taskGroupName);
            return TaskSpaceInverseDynamicsProblem();
        }

        if (priority == 1)
        {
            std::string weightProviderType = "ConstantWeightProvider";
            if (!taskGroup->getParameter("weight_provider_type", weightProviderType))
            {
                log()->warn("{} Unable to get the parameter 'weight_provider_type' for the task "
                            "in the group "
                            "'{}'. The default one will be used. Default: '{}'.",
                            logPrefix,
                            taskGroupName,
                            weightProviderType);
            }

            auto weightProvider = BipedalLocomotion::System::WeightProviderFactory::createInstance(
                weightProviderType);

            if (weightProvider == nullptr)
            {
                log()->error("{} The weight provider '{}' has not been registered.",
                             logPrefix,
                             weightProviderType);
                return TaskSpaceInverseDynamicsProblem();
            }

            if (!weightProvider->initialize(taskGroup))
            {
                log()->error("{} Unable to initialize the weight provider for the task in the "
                             "group "
                             "'{}'.",
                             logPrefix,
                             taskGroupName);
                return TaskSpaceInverseDynamicsProblem();
            }

            // the weight is considered only if priority is equal to 1
            if (!solver->addTask(taskInstance, taskGroupName, priority, weightProvider))
            {
                log()->error("{} Unable to add the task named '{}' in the solver.",
                             logPrefix,
                             taskGroupName);
                return TaskSpaceInverseDynamicsProblem();
            }

            weights[taskGroupName] = weightProvider;

        } else
        {
            if (!solver->addTask(taskInstance, taskGroupName, priority))
            {
                log()->error("{} Unable to add the task named '{}' in the solver.",
                             logPrefix,
                             taskGroupName);
                return TaskSpaceInverseDynamicsProblem();
            }
        }
    }

    if (!solver->finalize(variablesHandler))
    {
        log()->error("{} Unable to finalize the solver.", logPrefix);
        return TaskSpaceInverseDynamicsProblem();
    }

    TaskSpaceInverseDynamicsProblem problem;
    problem.tsid = std::move(solver);
    problem.variablesHandler = std::move(variablesHandler);
    problem.weights = std::move(weights);

    return problem;
}
