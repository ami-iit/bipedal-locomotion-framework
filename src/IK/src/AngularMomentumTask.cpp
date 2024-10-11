/**
 * @file AngularMomentumTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/IK/AngularMomentumTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;

bool AngularMomentumTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[AngularMomentumTask::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool AngularMomentumTask::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    if (!m_isInitialized)
    {
        log()->error("[AngularMomentumTask::setVariablesHandler] The task is not initialized. "
                     "Please call initialize method.");
        return false;
    }

    // get the variable
    if (!variablesHandler.getVariable(m_robotVelocityVariable.name, m_robotVelocityVariable))
    {
        log()->error("[AngularMomentumTask::setVariablesHandler] Unable to get the variable named "
                     "{}.",
                     m_robotVelocityVariable.name);
        return false;
    }

    // get the variable
    if (m_robotVelocityVariable.size != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        log()->error("[AngularMomentumTask::setVariablesHandler] The size of the robot velocity "
                     "variable is "
                     "different from the one expected. Expected size: {}. Given size: {}.",
                     m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize,
                     m_robotVelocityVariable.size);
        return false;
    }

    // resize the matrices
    m_A.resize(m_angularMomentumTaskSize, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_angularMomentumTaskSize);
    m_centroidalMomentumMatrix.resize(m_spatialVelocitySize, m_robotVelocityVariable.size);

    return true;
}

bool AngularMomentumTask::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[AngularMomentumTask::initialize]";

    std::string maskDescription = "";
    auto boolToString = [](bool b) { return b ? " true" : " false"; };
    for (const auto flag : m_mask)
    {
        maskDescription += boolToString(flag);
    }

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        log()->error("{} KinDynComputations object is not valid.", errorPrefix);
        return false;
    }

    if (m_kinDyn->getFrameVelocityRepresentation()
        != iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION)
    {
        log()->error("{} The task supports only quantities expressed in MIXED "
                     "representation. Please provide a KinDynComputations with Frame velocity "
                     "representation set to MIXED_REPRESENTATION.",
                     errorPrefix);
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("robot_velocity_variable_name", m_robotVelocityVariable.name))
    {
        log()->error("{} Error while retrieving the robot velocity variable.", errorPrefix);
        return false;
    }

    std::vector<bool> mask;
    if (!ptr->getParameter("mask", mask) || (mask.size() != m_angularVelocitySize))
    {
        log()->info("{} Unable to find the mask parameter. The default value is used:{}.",
                    errorPrefix,
                    maskDescription);
    } else
    {
        // covert an std::vector in a std::array
        std::copy(mask.begin(), mask.end(), m_mask.begin());
        // compute the DoFs associated to the task
        m_angularMomentumTaskSize = std::count(m_mask.begin(), m_mask.end(), true);

        // Update the mask description
        maskDescription.clear();
        for (const auto flag : m_mask)
        {
            maskDescription += boolToString(flag);
        }
    }

    m_description = "Angular momentum task. Mask:" + maskDescription + ".";

    m_isInitialized = true;

    return true;
}

bool AngularMomentumTask::update()
{
    using namespace iDynTree;

    m_isValid = false;

    if (!m_kinDyn->getCentroidalTotalMomentumJacobian(m_centroidalMomentumMatrix))
    {
        log()->error("[AngularMomentumTask::update] Unable to get the centroidal momentum matrix.");
        return m_isValid;
    }

    auto A(toEigen(this->subA(m_robotVelocityVariable)));
    if (m_angularMomentumTaskSize == m_angularVelocitySize)
    {
        A = m_centroidalMomentumMatrix.bottomRows<3>();
    } else
    {
        // take only the required components
        std::size_t index = 0;
        for (std::size_t i = 0; i < m_angularVelocitySize; i++)
        {
            if (m_mask[i])
            {
                // the first three elements of the centroidal momentum matrix are related to the
                // linear momentum
                A.row(index) = m_centroidalMomentumMatrix.row(i + 3);
                index++;
            }
        }
    }

    m_isValid = true;

    return m_isValid;
}

bool AngularMomentumTask::setSetPoint(Eigen::Ref<const Eigen::Vector3d> desiredAngularMomentum)
{

    if (m_angularMomentumTaskSize == m_angularVelocitySize)
    {
        m_b = desiredAngularMomentum;
    } else
    {
        // take only the required components
        std::size_t index = 0;
        for (std::size_t i = 0; i < m_angularVelocitySize; i++)
        {
            if (m_mask[i])
            {
                m_b(index) = desiredAngularMomentum(i);
                index++;
            }
        }
    }
    return true;
}

std::size_t AngularMomentumTask::size() const
{
    return m_angularMomentumTaskSize;
}

AngularMomentumTask::Type AngularMomentumTask::type() const
{
    return Type::equality;
}

bool AngularMomentumTask::isValid() const
{
    return m_isValid;
}
