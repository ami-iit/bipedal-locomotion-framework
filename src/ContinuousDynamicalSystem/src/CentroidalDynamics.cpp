/**
 * @file CentroidalDynamics.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/CentroidalDynamics.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::ContinuousDynamicalSystem;
using namespace BipedalLocomotion::ParametersHandler;

bool CentroidalDynamics::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[CentroidalDynamics::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is expired. Please call the function passing a "
                     "pointer pointing an already allocated memory.",
                     logPrefix);
        return false;
    }

    if (!ptr->getParameter("gravity", m_gravity))
    {
        log()->info("{} The gravity vector  not found. The default one will be used {}.",
                    logPrefix,
                    m_gravity.transpose());
    }

    if (!ptr->getParameter("mass", m_mass))
    {
        log()->info("{} The mass is not found. The default one will be used {} kg.",
                    logPrefix,
                    m_mass);
    }

    return true;
}

bool CentroidalDynamics::dynamics(const std::chrono::nanoseconds& time,
                                  StateDerivative& stateDerivative)
{
    using namespace BipedalLocomotion::GenericContainer::literals;

    const auto& [comPosition, comVelocity, angularMomentum] = m_state;
    auto& [comVelocityOut, comAcceleration, angularMomentumRate] = stateDerivative;

    comVelocityOut = comVelocity;
    comAcceleration = m_gravity;

    const auto& contacts = m_controlInput.get_from_hash<"contacts"_h>();
    const auto& externalDisturbance = m_controlInput.get_from_hash<"external_wrench"_h>();
    if (externalDisturbance)
    {
        comAcceleration += externalDisturbance.value().force();
        angularMomentumRate = externalDisturbance.value().torque();
    } else
    {
        angularMomentumRate.setZero();
    }

    for (const auto& [key, contact] : contacts)
    {
        for (const auto& corner : contact.corners)
        {
            comAcceleration.noalias() += 1 / m_mass * corner.force;
            angularMomentumRate.noalias()
                += (contact.pose.act(corner.position) - comPosition).cross(corner.force);
        }
    }

    return true;
}

bool CentroidalDynamics::setState(const State& state)
{
    m_state = state;
    return true;
}

const CentroidalDynamics::State& CentroidalDynamics::getState() const
{
    return m_state;
}

bool CentroidalDynamics::setControlInput(const Input& controlInput)
{
    m_controlInput = controlInput;
    return true;
}

void CentroidalDynamics::setGravityVector(const Eigen::Ref<const Eigen::Vector3d>& gravity)
{
    m_gravity = gravity;
}
