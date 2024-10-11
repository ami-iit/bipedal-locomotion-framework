/**
 * @file R3Task.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/TSID/R3Task.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;

bool R3Task::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    if (!m_isInitialized)
    {
        log()->error("[R3Task::setVariablesHandler] The task is not initialized. Please call "
                     "initialize method.");
        return false;
    }

    // get the variable
    if (!variablesHandler.getVariable(m_robotAccelerationVariable.name,
                                      m_robotAccelerationVariable))
    {
        log()->error("[R3Task::setVariablesHandler] Unable to get the variable named {}.",
                     m_robotAccelerationVariable.name);
        return false;
    }

    // get the variable
    if (m_robotAccelerationVariable.size
        != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        log()->error("[R3Task::setVariablesHandler] The size of the robot velocity variable is "
                     "different from the one expected. Expected size: {}. Given size: {}.",
                     m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize,
                     m_robotAccelerationVariable.size);
        return false;
    }

    // resize the matrices
    m_A.resize(m_DoFs, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_DoFs);
    m_jacobian.resize(m_spatialVelocitySize, m_robotAccelerationVariable.size);
    m_controllerOutput.resize(m_linearVelocitySize);

    return true;
}

bool R3Task::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[R3Task::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool R3Task::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr std::string_view errorPrefix = "[R3Task::initialize] ";

    std::string frameName = "Unknown";
    constexpr auto descriptionPrefix = "R3Task Optimal Control Element - Frame name: ";

    std::string maskDescription = "";
    auto boolToString = [](bool b) { return b ? " true" : " false"; };
    for (const auto flag : m_mask)
    {
        maskDescription += boolToString(flag);
    }

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        log()->error("{}, [{} {}] KinDynComputations object is not valid.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    if (m_kinDyn->getFrameVelocityRepresentation()
        != iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION)
    {
        log()->error("{}, [{} {}] The task supports only quantities expressed in MIXED "
                     "representation. Please provide a KinDynComputations with Frame velocity "
                     "representation set to MIXED_REPRESENTATION.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{}, [{} {}] The parameter handler is not valid.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    if (!ptr->getParameter("robot_acceleration_variable_name", m_robotAccelerationVariable.name))
    {
        log()->error("{}, [{} {}] Error while retrieving the robot velocity variable.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    if (!ptr->getParameter("frame_name", frameName)
        || (m_frameIndex = m_kinDyn->model().getFrameIndex(frameName))
               == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{}, [{} {}] Error while retrieving the frame that should be controlled.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    // set the gains for the R3 controller
    Eigen::Vector3d kpLinear, kdLinear;
    double bufferLinearScalar;

    if (ptr->getParameter("kp_linear", bufferLinearScalar))
    {
        kpLinear.setConstant(bufferLinearScalar);
    } else if (!ptr->getParameter("kp_linear", kpLinear))
    {
        log()->error("{}, [{} {}] Unable to get the proportional linear gain.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    // set the gains for the R3 controller
    if (ptr->getParameter("kd_linear", bufferLinearScalar))
    {
        kdLinear.setConstant(bufferLinearScalar);
    } else if (!ptr->getParameter("kd_linear", kdLinear))
    {
        log()->error("{}, [{} {}] Unable to get the derivative linear gain.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    m_R3Controller.setGains(kpLinear, kdLinear);

    std::vector<bool> mask;
    if (!ptr->getParameter("mask", mask) || (mask.size() != m_linearVelocitySize))
    {
        log()->info("{} [{} {}] Unable to find the mask parameter. The default value is used:{}.",
                    errorPrefix,
                    descriptionPrefix,
                    frameName,
                    maskDescription);
    } else
    {
        // covert an std::vector in a std::array
        std::copy(mask.begin(), mask.end(), m_mask.begin());
        // compute the DoFs associated to the task
        m_DoFs = std::count(m_mask.begin(), m_mask.end(), true);

        // Update the mask description
        maskDescription.clear();
        for (const auto flag : m_mask)
        {
            maskDescription += boolToString(flag);
        }
    }

    m_description = descriptionPrefix + frameName + " Mask:" + maskDescription + ".";

    m_isInitialized = true;

    return true;
}

bool R3Task::update()
{
    m_isValid = false;

    if (!m_isInitialized)
    {
        log()->error("[R3Task::update] Please call initialize() before update().");
        return m_isValid;
    }

    auto getGenericControllerOutput = [&](const auto& controller) {
        if (m_controllerMode == Mode::Enable)
            return controller.getControl().coeffs();
        else
            return controller.getFeedForward().coeffs();
    };

    m_controllerOutput = -iDynTree::toEigen(m_kinDyn->getFrameBiasAcc(m_frameIndex)).head<3>();

    m_R3Controller.setState(iDynTree::toEigen(
                                m_kinDyn->getWorldTransform(m_frameIndex).getPosition()),
                            iDynTree::toEigen(m_kinDyn->getFrameVel(m_frameIndex).getLinearVec3()));

    // update the controller output
    m_R3Controller.computeControlLaw();

    // get the output
    m_controllerOutput += getGenericControllerOutput(m_R3Controller);

    if (!m_kinDyn->getFrameFreeFloatingJacobian(m_frameIndex, m_jacobian))
    {
        log()->error("[R3Task::update] Unable to get the jacobian.");
        return m_isValid;
    }

    if (m_DoFs == m_linearVelocitySize)
    {
        m_b = m_controllerOutput;
        toEigen(this->subA(m_robotAccelerationVariable)) = m_jacobian.topRows<3>();

    } else
    {
        int index = 0;
        for (std::size_t i = 0; i < m_linearVelocitySize; i++)
        {
            if (!m_mask[i])
            {
                continue;
            }

            m_b(index) = m_controllerOutput(i);
            toEigen(this->subA(m_robotAccelerationVariable)).row(index) = m_jacobian.row(i);
            index++;
        }
    }

    m_isValid = true;
    return m_isValid;
}

bool R3Task::setSetPoint(Eigen::Ref<const Eigen::Vector3d> I_p_F,
                         Eigen::Ref<const Eigen::Vector3d> velocity, /** = Eigen::Vector3d::Zero() */
                         Eigen::Ref<const Eigen::Vector3d> acceleration /** = Eigen::Vector3d::Zero()*/)
{
    bool ok = true;
    ok = ok && m_R3Controller.setDesiredState(I_p_F, velocity);
    ok = ok && m_R3Controller.setFeedForward(acceleration);

    return ok;
}

std::size_t R3Task::size() const
{
    return m_DoFs;
}

R3Task::Type R3Task::type() const
{
    return Type::equality;
}

bool R3Task::isValid() const
{
    return m_isValid;
}

void R3Task::setTaskControllerMode(Mode mode)
{
    m_controllerMode = mode;
}

R3Task::Mode R3Task::getTaskControllerMode() const
{
    return m_controllerMode;
}

Eigen::Ref<const Eigen::VectorXd> R3Task::getControllerOutput() const
{
    return m_controllerOutput;
}
