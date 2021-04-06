/**
 * @file QPFixedBaseTSID.cpp
 * @authors Ines Sorrentino
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/TSID/QPFixedBaseTSID.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <OsqpEigen/OsqpEigen.h>

#include <BipedalLocomotion/TSID/SE3Task.h>
#include <BipedalLocomotion/TSID/JointDynamicsTask.h>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::TSID;
using namespace BipedalLocomotion::ParametersHandler;

struct QPFixedBaseTSID::Impl
{
    struct TaskWithPriority
    {
        std::shared_ptr<System::LinearTask> task;
        std::size_t priority;
        Eigen::VectorXd weight;
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
    bool isKinDynSet{false};

    TSIDState solution;

    System::VariablesHandler::VariableDescription robotAccelerationVariable;
    System::VariablesHandler::VariableDescription jointTorquesVariable;

    std::string baseLink;

    std::shared_ptr<SE3Task> baseSE3Task;
    std::shared_ptr<JointDynamicsTask> dynamicsTask;

    OsqpEigen::Solver solver; /**< Optimization solver. */

    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient;
    Eigen::MatrixXd constraintMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    bool initializeSolver()
    {
        constexpr auto logPrefix = "[QPFixedBaseTSID::Impl::initializeSolver]";
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
        constexpr auto logPrefix = "[QPFixedBaseTSID::Impl::updateSolver]";
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

QPFixedBaseTSID::QPFixedBaseTSID()
{
    m_pimpl = std::make_unique<QPFixedBaseTSID::Impl>();

    m_pimpl->baseSE3Task = std::make_shared<SE3Task>();

    m_pimpl->dynamicsTask = std::make_shared<JointDynamicsTask>();
}

QPFixedBaseTSID::~QPFixedBaseTSID() = default;

bool QPFixedBaseTSID::addTask(std::shared_ptr<System::LinearTask> task,
                              const std::string& taskName,
                              std::size_t priority,
                              std::optional<Eigen::Ref<const Eigen::VectorXd>> weight)
{
    constexpr auto logPrefix = "[QPFixedBaseTSID::addTask]";

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

    if (priority == 1 && !weight)
    {
        log()->error("{} - [Task name: '{}'] In case of priority equal to 1 the weight is "
                     "mandatory.",
                     logPrefix,
                     taskName);
        return false;
    }

    if (priority == 1 && task->type() == System::LinearTask::Type::inequality)
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

    if (priority == 1)
    {
        if (weight.value().size() != task->size())
        {
            log()->error("{} - [Task name: '{}'] The size of the weight is not coherent with the "
                         "size of the task. Expected: {}. Given: {}.",
                         logPrefix,
                         taskName,
                         m_pimpl->tasks[taskName].task->size(),
                         weight.value().size());

            // erase the task since it is not valid
            m_pimpl->tasks.erase(taskName);

            return false;
        }

        // add the weight
        m_pimpl->tasks[taskName].weight = weight.value();

        // add the task to the list of the element that are used to build the cost function
        m_pimpl->costs.push_back(m_pimpl->tasks[taskName]);
    }
    else
    {
        m_pimpl->constraints.push_back(m_pimpl->tasks[taskName]);
    }

    // if you add a new task you should call finalize again
    m_pimpl->isFinalized = false;

    return true;
}

std::vector<std::string> QPFixedBaseTSID::getTaskNames() const
{
    std::vector<std::string> tasksName;

    for (const auto& [name, task] : m_pimpl->tasks)
    {
        tasksName.push_back(name);
    }

    return tasksName;
}

std::weak_ptr<System::LinearTask> QPFixedBaseTSID::getTask(const std::string& name) const
{
    constexpr auto logPrefix = "[QPFixedBaseTSID::getTask]";
    auto task = m_pimpl->tasks.find(name);

    if (task == m_pimpl->tasks.end())
    {
        log()->warn("{} The task named {} does not exist. A nullptr will be returned.",
                    logPrefix,
                    name);
        return std::shared_ptr<System::LinearTask>(nullptr);
    }

    return task->second.task;
}

bool QPFixedBaseTSID::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    constexpr auto logPrefix = "[QPFixedBaseTSID::setKinDyn]";

    if (kinDyn == nullptr)
    {
        log()->error("{} The kinDyn object is not valid.", logPrefix);
        return false;
    }

    if (!m_pimpl->baseSE3Task->setKinDyn(kinDyn))
    {
        log()->error("{} Unable to set the kinDyn object for SE3Task on the base.", logPrefix);
        return false;
    }

    if (!m_pimpl->dynamicsTask->setKinDyn(kinDyn))
    {
        log()->error("{} Unable to set the kinDyn object for dynamicsTask.", logPrefix);
        return false;
    }

    m_pimpl->baseLink = kinDyn->getFloatingBase();

    m_pimpl->isKinDynSet = true;

    return true;
}

bool QPFixedBaseTSID::createBaseSE3Task()
{
    constexpr auto logPrefix = "[QPFixedBaseTSID::createBaseSE3Task]";

    auto baseSE3ParameterHandler = std::make_shared<StdImplementation>();
    baseSE3ParameterHandler->setParameter("robot_acceleration_variable_name",
                                          m_pimpl->robotAccelerationVariable.name);

    // This task is to consider the robot fixed base, so the controller gains are set to 0.
    baseSE3ParameterHandler->setParameter("kp_linear", 0.0);
    baseSE3ParameterHandler->setParameter("kd_linear", 0.0);
    baseSE3ParameterHandler->setParameter("kp_angular", 0.0);
    baseSE3ParameterHandler->setParameter("kd_angular", 0.0);
    baseSE3ParameterHandler->setParameter("frame_name", m_pimpl->baseLink);

    if (!m_pimpl->baseSE3Task->initialize(baseSE3ParameterHandler))
    {
        log()->error("{} Error initializing se3task on the base", logPrefix);
        return false;
    }

    m_pimpl->baseSE3Task->setSetPoint(manif::SE3d::Identity(),
                                      manif::SE3d::Tangent::Zero(),
                                      manif::SE3d::Tangent::Zero());

    return true;
}

bool QPFixedBaseTSID::createDynamicsTask()
{
    constexpr auto logPrefix = "[QPFixedBaseTSID::createDynamicsTask]";

    auto dynamicsParameterHandler = std::make_shared<StdImplementation>();

    dynamicsParameterHandler->setParameter("robot_acceleration_variable_name",
                                           m_pimpl->robotAccelerationVariable.name);
    dynamicsParameterHandler->setParameter("joint_torques_variable_name",
                                           m_pimpl->jointTorquesVariable.name);
    dynamicsParameterHandler->setParameter("max_number_of_contacts", 1); // We assume the external
                                                                         // wrench acting on only
                                                                         // the base link.

    auto contactGroup = std::make_shared<StdImplementation>();
    contactGroup->setParameter("variable_name", "base_contact_wrench");
    contactGroup->setParameter("frame_name", m_pimpl->baseLink);
    if (!dynamicsParameterHandler->setGroup("CONTACT_0", contactGroup))
    {
        log()->error("{} Error setting group for base contact.", logPrefix);
        return false;
    }

    if (!m_pimpl->dynamicsTask->initialize(dynamicsParameterHandler))
    {
        log()->error("{} Error initializing dynamics task", logPrefix);
        return false;
    }

    return true;
}

bool QPFixedBaseTSID::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[QPFixedBaseTSID::initialize]";

    if (!m_pimpl->isKinDynSet)
    {
        log()->error("{} The object kinDyn is not set for the base SE3Task.", logPrefix);
        return false;
    }

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

    if (!createBaseSE3Task())
    {
        log()->error("{} Error creating se3task for the base.", logPrefix);
        return false;
    }
    if (!createDynamicsTask())
    {
        log()->error("{} Error creating task for the joint dynamics.", logPrefix);
        return false;
    }
    constexpr std::size_t highPriority = 0;
    this->addTask(m_pimpl->baseSE3Task, "base_se3_task", highPriority);
    this->addTask(m_pimpl->dynamicsTask, "dynamics_task", highPriority);

    m_pimpl->isInitialized = true;

    return true;
}

bool QPFixedBaseTSID::finalize(const System::VariablesHandler& handler)
{
    constexpr auto logPrefix = "[QPFixedBaseTSID::finalize]";

    System::VariablesHandler tmpHandler = handler;
    // The vector considering the contact wrench on the base link contains 6 elements, the 3 forces
    // in the 3 directions and the 3 moments about the 3 axes.
    if (!tmpHandler.addVariable("base_contact_wrench", 6))
    {
        log()->error("{} Error while adding base contact wrench variable to the handler",
                     logPrefix);
        return false;
    }

    // clear the solver
    m_pimpl->solver.clearSolver();

    // set the some internal parameter of osqp-eigen
    m_pimpl->solver.settings()->setVerbosity(m_pimpl->isVerbose);

    // set the variable handler for all the tasks
    m_pimpl->numberOfConstraints = 0;
    for (auto& [name, task] : m_pimpl->tasks)
    {
        if (!task.task->setVariablesHandler(tmpHandler))
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
        cost.get().tmp.resize(tmpHandler.getNumberOfVariables(), cost.get().weight.size());
    }

    m_pimpl->solver.data()->setNumberOfVariables(tmpHandler.getNumberOfVariables());
    m_pimpl->solver.data()->setNumberOfConstraints(m_pimpl->numberOfConstraints);

    // resize matrices
    m_pimpl->hessian.resize(tmpHandler.getNumberOfVariables(), tmpHandler.getNumberOfVariables());
    m_pimpl->gradient.resize(tmpHandler.getNumberOfVariables());

    m_pimpl->constraintMatrix.resize(m_pimpl->numberOfConstraints,
                                     tmpHandler.getNumberOfVariables());
    m_pimpl->upperBound.resize(m_pimpl->numberOfConstraints);
    m_pimpl->lowerBound.resize(m_pimpl->numberOfConstraints);

    if (!tmpHandler.getVariable(m_pimpl->robotAccelerationVariable.name,
                         m_pimpl->robotAccelerationVariable))
    {
        log()->error("{} Error while retrieving the robot acceleration variable.", logPrefix);
        return false;
    }

    if (!tmpHandler.getVariable(m_pimpl->jointTorquesVariable.name, m_pimpl->jointTorquesVariable))
    {
        log()->error("{} Error while retrieving the joint torques variable.", logPrefix);
        return false;
    }

    m_pimpl->isFinalized = true;

    return true;
}

bool QPFixedBaseTSID::advance()
{
    constexpr auto logPrefix = "[QPFixedBaseTSID::advance]";

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
        cost.get().tmp.noalias() = A.transpose() * cost.get().weight.asDiagonal();
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
    if (!m_pimpl->solver.solve())
    {
        log()->error("{} Unable to to solve the problem.", logPrefix);
        return false;
    }

    // retrieve the solution
    constexpr std::size_t spatialAccelerationSize = 6;
    const std::size_t joints = m_pimpl->robotAccelerationVariable.size - spatialAccelerationSize;

    m_pimpl->solution.jointTorques
        = m_pimpl->solver.getSolution().segment(m_pimpl->robotAccelerationVariable.offset
                                                    + m_pimpl->robotAccelerationVariable.size,
                                                joints);

    m_pimpl->isValid = true;

    return true;
}

const TSIDState& QPFixedBaseTSID::getOutput() const
{
    return m_pimpl->solution;
}

bool QPFixedBaseTSID::isOutputValid() const
{
    return m_pimpl->isValid;
}
