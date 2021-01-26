/**
 * @file BaseDynamicsTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/TSID/BaseDynamicsTask.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;

bool BaseDynamicsTask::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> paramHandler,
                                  const System::VariablesHandler& variablesHandler)
{
    constexpr std::string_view errorPrefix = "[BaseDynamicsTask::initialize] ";

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        std::cerr << errorPrefix << "KinDynComputations object is not valid." << std::endl;
        return false;
    }

    if (m_kinDyn->getFrameVelocityRepresentation()
        != iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION)
    {
        std::cerr << errorPrefix
                  << "The task supports only quantities expressed in MIXED representation. "
                     "Please provide a KinDynComputations with Frame velocity representation set "
                     "to MIXED_REPRESENTATION."
                  << std::endl;
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        std::cerr << errorPrefix << "The parameter handler is not valid." << std::endl;
        return false;
    }

    std::string robotAccelerationVariableName;
    if (!ptr->getParameter("robot_acceleration_variable_name", robotAccelerationVariableName)
        || !variablesHandler.getVariable(robotAccelerationVariableName,
                                         m_robotAccelerationVariable))
    {
        std::cerr << errorPrefix << "Error while retrieving the robot acceleration variable."
                  << std::endl;
        return false;
    }

    if (m_robotAccelerationVariable.size
        != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        std::cerr << errorPrefix << "Error while retrieving the robot acceleration variable."
                  << std::endl;
        return false;
    }

    int numberOfContacts = 0;
    ptr->getParameter("max_number_of_contacts", numberOfContacts);

    for (int i = 0; i < numberOfContacts; i++)
    {
        auto groupWeak = ptr->getGroup("CONTACT_" + std::to_string(i));
        auto group = groupWeak.lock();
        if (group == nullptr)
        {
            std::cerr << errorPrefix << "The group named: CONTACT_" << i << " do not exist."
                      << std::endl;
            return false;
        }

        std::string frameName;
        ContactWrench tmp;
        if (!group->getParameter("frame_name", frameName)
            || (tmp.frameIndex = m_kinDyn->model().getFrameIndex(frameName))
                   == iDynTree::FRAME_INVALID_INDEX)
        {
            std::cerr << errorPrefix
                      << "Error while retrieving the frame associated to the CONTACT_" << i << "."
                      << std::endl;
            return false;
        }

        std::string variableName;
        if (!group->getParameter("variable_name", variableName)
            || !variablesHandler.getVariable(variableName, tmp.variable))
        {
            std::cerr << errorPrefix
                      << "Error while retrieving the variable name associated to the CONTACT_" << i
                      << "." << std::endl;
            return false;
        }

        if (tmp.variable.size != m_spatialVelocitySize)
        {
            std::cerr << errorPrefix << "The variable size associated to the CONTACT_" << i
                      << " is different from " << m_spatialVelocitySize << "." << std::endl;
            return false;
        }

        m_contactWrenches.push_back(tmp);
    }

    // set the description
    m_description = "Base dynamics Task.";

    // resize the matrices
    m_A.resize(m_spatialVelocitySize, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_spatialVelocitySize);

    m_jacobian.resize(m_spatialVelocitySize, m_robotAccelerationVariable.size);
    m_massMatrix.resize(m_robotAccelerationVariable.size, m_robotAccelerationVariable.size);
    m_generalizedBiasForces.resize(m_robotAccelerationVariable.size);

    return true;
}

bool BaseDynamicsTask::update()
{
    if (!m_kinDyn->generalizedBiasForces(m_generalizedBiasForces))
    {
        std::cerr << "[BaseDynamicsTask::update] Unable to get the bias forces." << std::endl;
        return false;
    }

    if (!m_kinDyn->getFreeFloatingMassMatrix(m_massMatrix))
    {
        std::cerr << "[BaseDynamicsTask::update] Unable to get the mass matrix." << std::endl;
        return false;
    }

    m_b = m_generalizedBiasForces.head<m_spatialVelocitySize>();
    iDynTree::toEigen(this->subA(m_robotAccelerationVariable))
        = -m_massMatrix.topRows<m_spatialVelocitySize>();

    for (const auto& contactWrench : m_contactWrenches)
    {
        if (!m_kinDyn->getFrameFreeFloatingJacobian(contactWrench.frameIndex, m_jacobian))
        {
            std::cerr << "[BaseDynamicsTask::update] Unable to get contact wrench associated to "
                         "frame named "
                      << m_kinDyn->model().getFrameName(contactWrench.frameIndex) << "."
                      << std::endl;
            return false;
        }

        iDynTree::toEigen(this->subA(contactWrench.variable))
            = m_jacobian.transpose().topRows<m_spatialVelocitySize>();
    }

    return true;
}
