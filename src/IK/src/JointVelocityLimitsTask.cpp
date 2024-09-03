/**
 * @file JointVelocityLimitsTask.cpp
 * @authors Davide Gorbani
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <types.h>
#include <vector>

#include <OsqpEigen/Constants.hpp>

#include <BipedalLocomotion/IK/JointVelocityLimitsTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::IK;

bool JointVelocityLimitsTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[JointVelocityLimitsTask::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;

    // populate the limits with the one retrieved from the model
    m_upperLimits
        = Eigen::VectorXd::Constant(m_kinDyn->getNrOfDegreesOfFreedom(), OsqpEigen::INFTY);
    m_lowerLimits
        = Eigen::VectorXd::Constant(m_kinDyn->getNrOfDegreesOfFreedom(), -OsqpEigen::INFTY);

    return true;
}

bool JointVelocityLimitsTask::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    constexpr auto errorPrefix = "[JointVelocityLimitsTask::setVariablesHandler]";

    System::VariablesHandler::VariableDescription robotVelocityVariable;

    if (!m_isInitialized)
    {
        log()->error("{} The task is not initialized. Please call initialize method.", errorPrefix);
        return false;
    }

    if (!variablesHandler.getVariable(m_robotVelocityVariableName, robotVelocityVariable))
    {
        log()->error("{} Error while retrieving the robot velocity variable.", errorPrefix);
        return false;
    }

    if (robotVelocityVariable.size != m_kinDyn->getNrOfDegreesOfFreedom() + 6)
    {
        log()->error("{} The size of the robot velocity variable does not match with the one "
                     "stored in kinDynComputations object. Expected: {}. Given: {}",
                     errorPrefix,
                     m_kinDyn->getNrOfDegreesOfFreedom() + 6,
                     robotVelocityVariable.size);
        return false;
    }

    // resize the matrices
    m_A.resize(2 * m_kinDyn->getNrOfDegreesOfFreedom(), variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(2 * m_kinDyn->getNrOfDegreesOfFreedom());
    m_b.head(m_kinDyn->getNrOfDegreesOfFreedom()).noalias() = m_upperLimits;
    m_b.tail(m_kinDyn->getNrOfDegreesOfFreedom()).noalias() = -m_lowerLimits;

    // the submatrix associated to the robot velocity
    //      _                _
    // A = | 0_{6x6} +I_{nxn} |
    //     | 0_{6x6} -I_{nxn} |
    //     |_                _|
    iDynTree::toEigen(this->subA(robotVelocityVariable))
        .topRightCorner(m_kinDyn->getNrOfDegreesOfFreedom(), m_kinDyn->getNrOfDegreesOfFreedom())
        .diagonal()
        .setConstant(1);
    iDynTree::toEigen(this->subA(robotVelocityVariable))
        .bottomRightCorner(m_kinDyn->getNrOfDegreesOfFreedom(), m_kinDyn->getNrOfDegreesOfFreedom())
        .diagonal()
        .setConstant(-1);

    return true;
}

bool JointVelocityLimitsTask::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[JointVelocityLimitsTask::initialize]";

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        log()->error("{} KinDynComputations object is not valid.", errorPrefix);
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("robot_velocity_variable_name", m_robotVelocityVariableName))
    {
        log()->error("{} Error while retrieving the robot velocity variable.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("upper_limits", m_upperLimits)
        || !ptr->getParameter("lower_limits", m_lowerLimits))
    {
        log()->error("{} Error while retrieving the 'upper_limits' and/or the 'lower_limit' "
                     "parameters.",
                     errorPrefix);
        return false;
    }

    // set the description
    m_description = "Joint limits task";

    m_isInitialized = true;

    return true;
}

bool JointVelocityLimitsTask::update()
{
    constexpr auto errorPrefix = "[JointVelocityLimitsTask::update]";

    m_isValid = false;

    m_b.head(m_kinDyn->getNrOfDegreesOfFreedom()).noalias() = m_upperLimits;
    m_b.tail(m_kinDyn->getNrOfDegreesOfFreedom()).noalias() = -m_lowerLimits;

    m_isValid = true;
    return m_isValid;
}

std::size_t JointVelocityLimitsTask::size() const
{
    constexpr auto errorMessage = "[JointVelocityLimitsTask::size] Please call setKinDyn method "
                                  "before. "
                                  "A size equal to zero will be returned.";

    assert((m_kinDyn != nullptr) && errorMessage);

    if (m_kinDyn == nullptr)
    {
        log()->warn(errorMessage);
        return 0;
    }
    return 2 * m_kinDyn->getNrOfDegreesOfFreedom();
}

JointVelocityLimitsTask::Type JointVelocityLimitsTask::type() const
{
    return Type::inequality;
}

bool JointVelocityLimitsTask::isValid() const
{
    return m_isValid;
}
