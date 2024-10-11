/**
 * @file JointDynamicsTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TSID/JointDynamicsTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;

bool JointDynamicsTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[JointDynamicsTask::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool JointDynamicsTask::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    constexpr auto errorPrefix = "[JointDynamicsTask::setVariablesHandler]";

    if (!m_isInitialized)
    {
        log()->error("{} The task is not initialized. Please call initialize method.", errorPrefix);
        return false;
    }

    if (!variablesHandler.getVariable(m_robotAccelerationVariable.name,
                                      m_robotAccelerationVariable))
    {
        log()->error("{} Error while retrieving the robot acceleration variable.", errorPrefix);
        return false;
    }

    if (!variablesHandler.getVariable(m_jointsTorqueVariable.name, m_jointsTorqueVariable))
    {
        log()->error("{} Error while retrieving the joint torques variable.", errorPrefix);
        return false;
    }

    if (m_robotAccelerationVariable.size
        != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        log()->error("{} The size of the robot acceleration variable does not match with the one "
                     "stored in kinDynComputations object. Expected: {}. Given: {}",
                     errorPrefix,
                     m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize,
                     m_robotAccelerationVariable.size);
        return false;
    }

    if (m_jointsTorqueVariable.size != m_kinDyn->getNrOfDegreesOfFreedom())
    {
        log()->error("{} The size of the joint torque variable does not match with the one "
                     "stored in kinDynComputations object. Expected: {}. Given: {}",
                     errorPrefix,
                     m_kinDyn->getNrOfDegreesOfFreedom(),
                     m_jointsTorqueVariable.size);
        return false;
    }

    for (auto& contact : m_contactWrenches)
    {
        if (!variablesHandler.getVariable(contact.variable.name, contact.variable))
        {
            log()->error("{} Error while retrieving the contact variable named {}.",
                         errorPrefix,
                         contact.variable.name);
            return false;
        }

        if (contact.variable.size != m_spatialVelocitySize)
        {
            log()->error("{} The variable size associated to the contact named {} is different "
                         "from {}.",
                         errorPrefix,
                         contact.variable.name,
                         m_spatialVelocitySize);
            return false;
        }
    }

    // resize the matrices
    m_A.resize(m_kinDyn->getNrOfDegreesOfFreedom(), variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_kinDyn->getNrOfDegreesOfFreedom());

    iDynTree::toEigen(this->subA(m_jointsTorqueVariable)).setIdentity();

    return true;
}

bool JointDynamicsTask::setMassMatrixRegularization(const Eigen::Ref<const Eigen::MatrixXd>& matrix)
{
    constexpr auto logPrefix = "[JointDynamicsTask::"
                               "setMassMatrixRegularization]";

    if ((m_kinDyn == nullptr) || (!m_kinDyn->isValid()))
    {
        log()->error("{} Please call setKinDyn() before.", logPrefix);
        return false;
    }

    if ((m_kinDyn->getNrOfDegreesOfFreedom() != matrix.rows()) || (matrix.cols() != matrix.rows()))
    {
        const auto rightSize = m_kinDyn->getNrOfDegreesOfFreedom();

        log()->error("{} The size of the regularization matrix is not correct. The correct size "
                     "is:  {} x {}. While the input of the function is a {} x {} matrix.",
                     logPrefix,
                     rightSize,
                     rightSize,
                     matrix.rows(),
                     matrix.cols());
        return false;
    }

    m_massMatrixRegularizationTerm = matrix;
    m_useMassMatrixRegularizationTerm = true;

    return true;
}

bool JointDynamicsTask::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[JointDynamicsTask::initialize]";

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        log()->error("{} KinDynComputations object is not valid.", errorPrefix);
        return false;
    }

    if (m_kinDyn->getFrameVelocityRepresentation()
        != iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION)
    {
        log()->error("{} The task supports only quantities expressed in MIXED representation. "
                     "Please provide a KinDynComputations with Frame velocity representation set "
                     "to MIXED_REPRESENTATION.",
                     errorPrefix);
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("robot_acceleration_variable_name", m_robotAccelerationVariable.name))
    {
        log()->error("{} Error while retrieving the robot acceleration variable.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("joint_torques_variable_name", m_jointsTorqueVariable.name))
    {
        log()->error("{} Error while retrieving the robot torque variable.", errorPrefix);
        return false;
    }

    int numberOfContacts = 0;
    ptr->getParameter("max_number_of_contacts", numberOfContacts);

    m_contactWrenches.resize(numberOfContacts);

    for (int i = 0; i < numberOfContacts; i++)
    {
        auto groupWeak = ptr->getGroup("CONTACT_" + std::to_string(i));
        auto group = groupWeak.lock();
        if (group == nullptr)
        {
            log()->error("{} The group named CONTACT_{} does not exist.", errorPrefix, i);
            return false;
        }

        std::string frameName;
        if (!group->getParameter("frame_name", frameName)
            || (m_contactWrenches[i].frameIndex = m_kinDyn->model().getFrameIndex(frameName))
                   == iDynTree::FRAME_INVALID_INDEX)
        {
            log()->error("{} Error while retrieving the frame associated to the CONTACT_{}.",
                         errorPrefix,
                         i);
            return false;
        }

        if (!group->getParameter("variable_name", m_contactWrenches[i].variable.name))
        {
            log()->error("{} Error while retrieving the variable name associated to the "
                         "CONTACT_{}.",
                         errorPrefix,
                         i);
            return false;
        }
    }

    // set the description
    m_description = "Joint dynamics Task.";

    // reset the jacobian
    m_jacobian.resize(m_spatialVelocitySize,
                      m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize);
    m_massMatrix.resize(m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize,
                        m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize);
    m_generalizedBiasForces.resize(m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize);

    m_isInitialized = true;

    return true;
}

bool JointDynamicsTask::update()
{
    constexpr auto errorPrefix = "[JointDynamicsTask::update]";

    m_isValid = false;

    if (!m_kinDyn->generalizedBiasForces(m_generalizedBiasForces))
    {
        log()->error("{} Unable to get the bias forces.", errorPrefix);
        return false;
    }

    if (!m_kinDyn->getFreeFloatingMassMatrix(m_massMatrix))
    {
        log()->error("{} Unable to get the mass matrix.", errorPrefix);
        return false;
    }

    m_b = m_generalizedBiasForces.tail(m_kinDyn->getNrOfDegreesOfFreedom());

    if (m_useMassMatrixRegularizationTerm)
    {
        iDynTree::toEigen(this->subA(m_robotAccelerationVariable))
            = -(m_massMatrix.bottomRows(m_kinDyn->getNrOfDegreesOfFreedom())
                + m_massMatrixRegularizationTerm);
    } else
    {
        iDynTree::toEigen(this->subA(m_robotAccelerationVariable))
            = -m_massMatrix.bottomRows(m_kinDyn->getNrOfDegreesOfFreedom());
    }

    for (const auto& contactWrench : m_contactWrenches)
    {
        if (!m_kinDyn->getFrameFreeFloatingJacobian(contactWrench.frameIndex, m_jacobian))
        {
            log()->error("{} Unable to get contact wrench associated to frame named {}.",
                         errorPrefix,
                         m_kinDyn->model().getFrameName(contactWrench.frameIndex));
            return false;
        }

        iDynTree::toEigen(this->subA(contactWrench.variable))
            = m_jacobian.transpose().bottomRows(m_kinDyn->getNrOfDegreesOfFreedom());
    }

    m_isValid = true;
    return m_isValid;
}

std::size_t JointDynamicsTask::size() const
{
    constexpr auto errorMessage = "[JointTrackingTask::size] Please call setKinDyn method before. "
                                  "A size equal to zero will be returned.";

    assert((m_kinDyn != nullptr) && errorMessage);

    if (m_kinDyn == nullptr)
    {
        log()->warn(errorMessage);
        return 0;
    }
    return m_kinDyn->getNrOfDegreesOfFreedom();
}

JointDynamicsTask::Type JointDynamicsTask::type() const
{
    return Type::equality;
}

bool JointDynamicsTask::isValid() const
{
    return m_isValid;
}
