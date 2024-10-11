/**
 * @file CoMTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/TSID/CoMTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;

bool CoMTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[CoMTask::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool CoMTask::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    if (!m_isInitialized)
    {
        log()->error("[CoMTask::setVariablesHandler] The task is not initialized. Please call "
                     "initialize method.");
        return false;
    }

    // get the variable
    if (!variablesHandler.getVariable(m_robotAccelerationVariable.name,
                                      m_robotAccelerationVariable))
    {
        log()->error("[CoMTask::setVariablesHandler] Unable to get the variable named {}.",
                     m_robotAccelerationVariable.name);
        return false;
    }

    // get the variable
    if (m_robotAccelerationVariable.size
        != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        log()->error("[CoMTask::setVariablesHandler] The size of the robot velocity variable is "
                     "different from the one expected. Expected size: {}. Given size: {}.",
                     m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize,
                     m_robotAccelerationVariable.size);
        return false;
    }

    // resize the matrices
    m_A.resize(m_DoFs, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_DoFs);
    m_b.setZero();
    m_jacobian.resize(m_linearVelocitySize, m_robotAccelerationVariable.size);

    return true;
}

bool CoMTask::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[CoMTask::initialize]";

    m_description = "CoMTask Task.";

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        log()->error("{} [{}] KinDynComputations object is not valid.", errorPrefix, m_description);

        return false;
    }

    if (m_kinDyn->getFrameVelocityRepresentation()
        != iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION)
    {
        log()->error("{} [{}] task supports only quantities expressed in MIXED "
                     "representation. Please provide a KinDynComputations with Frame velocity "
                     "representation set to MIXED_REPRESENTATION.",
                     errorPrefix,
                     m_description);
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} [{}] The parameter handler is not valid.", errorPrefix, m_description);
        return false;
    }

    if (!ptr->getParameter("robot_acceleration_variable_name", m_robotAccelerationVariable.name))
    {
        log()->error("{} [{}] while retrieving the robot velocity variable.",
                     errorPrefix,
                     m_description);
        return false;
    }

    double scalarBuffer;
    Eigen::Vector3d kpLinear, kdLinear;
    if (ptr->getParameter("kp_linear", scalarBuffer))
    {
        kpLinear.setConstant(scalarBuffer);
    } else if (!ptr->getParameter("kp_linear", kpLinear))
    {
        log()->error("{}, [{}] Unable to get the proportional gain.", errorPrefix, m_description);
        return false;
    }

    if (ptr->getParameter("kd_linear", scalarBuffer))
    {
        kdLinear.setConstant(scalarBuffer);
    } else if (!ptr->getParameter("kd_linear", kdLinear))
    {
        log()->error("{}, [{}] Unable to get the derivative gain.", errorPrefix, m_description);
        return false;
    }

    m_R3Controller.setGains(std::move(kpLinear), std::move(kdLinear));

    std::string maskDescription = "";
    auto boolToString = [](bool b) { return b ? " true" : " false"; };
    for(const auto flag : m_mask)
    {
        maskDescription += boolToString(flag);
    }

    std::vector<bool> mask;
    if (!ptr->getParameter("mask", mask) || (mask.size() != m_linearVelocitySize))
    {
        log()->info("{} [{}] Unable to find the mask parameter. The default value is used:{}.",
                    errorPrefix,
                    m_description,
                    maskDescription);
    }
    else
    {
        // convert an std::vector in a std::array
        std::copy(mask.begin(), mask.end(), m_mask.begin());
        // compute the DoFs associated to the task
        m_DoFs = std::count(m_mask.begin(), m_mask.end(), true);

        // Update the mask description
        maskDescription.clear();
        for(const auto flag : m_mask)
        {
            maskDescription += boolToString(flag);
        }
    }

    m_description += " Mask:" + maskDescription;

    m_isInitialized = true;

    return true;
}

bool CoMTask::update()
{
    using namespace BipedalLocomotion::Conversions;
    using namespace iDynTree;

    m_isValid = false;

    // set the state
    m_R3Controller.setState(toEigen(m_kinDyn->getCenterOfMassPosition()),
                            toEigen(m_kinDyn->getCenterOfMassVelocity()));

    // update the controller
    m_R3Controller.computeControlLaw();

    Eigen::Vector3d bFullDof = m_R3Controller.getControl().coeffs()
        - iDynTree::toEigen(m_kinDyn->getCenterOfMassBiasAcc());

    // if we want to control all 3 DoF we avoid to lose performances
    if (m_DoFs == m_linearVelocitySize)
    {
        m_b = bFullDof;

        // get the CoM jacobian
        if (!m_kinDyn->getCenterOfMassJacobian(this->subA(m_robotAccelerationVariable)))
        {
            log()->error("[CoMTask::update] Unable to get the jacobian.");
            return m_isValid;
        }
    } else
    {
        // get the CoM jacobian
        if (!m_kinDyn->getCenterOfMassJacobian(m_jacobian))
        {
            log()->error("[CoMTask::update] Unable to get the jacobian.");
            return m_isValid;
        }

        int index = 0;
        for (std::size_t i = 0; i < m_linearVelocitySize; i++)
        {
            if (m_mask[i])
            {
                m_b(index) = bFullDof(i);
                toEigen(this->subA(m_robotAccelerationVariable)).row(index) = m_jacobian.row(i);
                index++;
            }
        }
    }

    // A and b are now valid
    m_isValid = true;
    return m_isValid;
}

bool CoMTask::setSetPoint(Eigen::Ref<const Eigen::Vector3d> position,
                          Eigen::Ref<const Eigen::Vector3d> velocity,
                          Eigen::Ref<const Eigen::Vector3d> acceleration)
{
    bool ok = true;
    ok = ok && m_R3Controller.setDesiredState(position, velocity);
    ok = ok && m_R3Controller.setFeedForward(acceleration);
    return ok;
}

std::size_t CoMTask::size() const
{
    return m_DoFs;
}

CoMTask::Type CoMTask::type() const
{
    return Type::equality;
}

bool CoMTask::isValid() const
{
    return m_isValid;
}
