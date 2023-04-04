/**
 * @file FloatingBaseSystemKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ContinuousDynamicalSystem;
using namespace BipedalLocomotion::ParametersHandler;

bool FloatingBaseSystemKinematics::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[FloatingBaseSystemKinematics::initialize]";

    auto ptr = handler.lock();
    if (ptr != nullptr)
    {
        double baumgarte = 0;
        if (ptr->getParameter("rho", baumgarte))
        {
            log()->info("{} The rho parameter of the Baumgarte stabilization is no more required. "
                        "The integration is computed on the manifold directly",
                        logPrefix);
        }
    }

    return true;
}

bool FloatingBaseSystemKinematics::dynamics(const std::chrono::nanoseconds& time,
                                            StateDerivative& stateDerivative)
{
    using namespace BipedalLocomotion::GenericContainer::literals;

    // get the state
    const Eigen::Vector3d& basePosition = m_state.get_from_hash<"p"_h>();
    const manif::SO3d& baseRotation = m_state.get_from_hash<"R"_h>();
    const Eigen::VectorXd& jointPositions = m_state.get_from_hash<"s"_h>();

    // get the state derivative
    Eigen::Vector3d& baseLinearVelocity = stateDerivative.get_from_hash<"dp"_h>();
    manif::SO3d::Tangent& baseAngularVelocity = stateDerivative.get_from_hash<"omega"_h>();
    Eigen::VectorXd& jointVelocityOutput = stateDerivative.get_from_hash<"ds"_h>();

    const Eigen::Matrix<double, 6, 1>& baseTwist = m_controlInput.get_from_hash<"twist"_h>();
    const Eigen::VectorXd& jointVelocity = m_controlInput.get_from_hash<"ds"_h>();

    // check the size of the vectors
    if (jointVelocity.size() != jointPositions.size())
    {
        log()->error("[FloatingBaseSystemKinematics::dynamics] Wrong size of the vectors.");
        return false;
    }

    // compute the base linear velocity
    baseLinearVelocity = baseTwist.head<3>();

    // here we assume that the velocity is expressed using the mixed representation
    baseAngularVelocity = baseTwist.tail<3>();

    jointVelocityOutput = jointVelocity;

    return true;
}

bool FloatingBaseSystemKinematics::setState(const FloatingBaseSystemKinematics::State& state)
{
    m_state = state;
    return true;
}

const FloatingBaseSystemKinematics::State& FloatingBaseSystemKinematics::getState() const
{
    return m_state;
}

bool FloatingBaseSystemKinematics::setControlInput(const Input& controlInput)
{
    m_controlInput = controlInput;
    return true;
}
