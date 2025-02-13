/**
 * @file FloatingBaseSystemAccelerationKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemAccelerationKinematics.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ContinuousDynamicalSystem;
using namespace BipedalLocomotion::ParametersHandler;

bool FloatingBaseSystemAccelerationKinematics::initialize(
    std::weak_ptr<const IParametersHandler> handler)
{
    return true;
}

bool FloatingBaseSystemAccelerationKinematics::dynamics(const std::chrono::nanoseconds& time,
                                                        StateDerivative& stateDerivative)
{
    using namespace BipedalLocomotion::GenericContainer::literals;

    // get the state
    const Eigen::Vector3d& basePositionState = m_state.get_from_hash<"p"_h>();
    const manif::SO3d& baseRotationState = m_state.get_from_hash<"R"_h>();
    const Eigen::VectorXd& jointPositionsState = m_state.get_from_hash<"s"_h>();
    const Eigen::Vector3d& baseLinearVelocityState = m_state.get_from_hash<"dp"_h>();
    const manif::SO3d::Tangent& baseAngularVelocityState = m_state.get_from_hash<"omega"_h>();
    const Eigen::VectorXd& jointVelocitiesState = m_state.get_from_hash<"ds"_h>();

    // get the state derivative
    Eigen::Vector3d& baseLinearVelocityStateDerivative = stateDerivative.get_from_hash<"dp"_h>();
    manif::SO3d::Tangent& baseAngularVelocityStateDerivative
        = stateDerivative.get_from_hash<"omega"_h>();
    Eigen::VectorXd& jointVelocitiesStateDerivative = stateDerivative.get_from_hash<"ds"_h>();
    Eigen::Vector3d& baseLinearAccelerationStateDerivative
        = stateDerivative.get_from_hash<"ddp"_h>();
    manif::SO3d::Tangent& baseAngularAccelerationStateDerivative
        = stateDerivative.get_from_hash<"domega"_h>();
    Eigen::VectorXd& jointAccelerationsStateDerivative = stateDerivative.get_from_hash<"dds"_h>();

    // get the control input
    const Eigen::Matrix<double, 6, 1>& baseAcceleration
        = m_controlInput.get_from_hash<"dtwist"_h>();
    const Eigen::VectorXd& jointAcceleration = m_controlInput.get_from_hash<"dds"_h>();

    // check the size of the joint vectors
    if (jointAcceleration.size() != jointPositionsState.size()
        || jointVelocitiesState.size() != jointPositionsState.size()
        || jointVelocitiesStateDerivative.size() != jointPositionsState.size()
        || jointAccelerationsStateDerivative.size() != jointPositionsState.size())
    {
        log()->error("[FloatingBaseSystemAccelerationKinematics::dynamics] Wrong size of the "
                     "vectors.");
        return false;
    }

    baseLinearVelocityStateDerivative = baseLinearVelocityState;
    baseAngularVelocityStateDerivative = baseAngularVelocityState;
    jointVelocitiesStateDerivative = jointVelocitiesState;

    // we assume the base acceleration is expressed in the mixed representation
    baseLinearAccelerationStateDerivative = baseAcceleration.head<3>();
    baseAngularAccelerationStateDerivative = baseAcceleration.tail<3>();
    jointAccelerationsStateDerivative = jointAcceleration;

    return true;
}

bool FloatingBaseSystemAccelerationKinematics::setState(
    const FloatingBaseSystemAccelerationKinematics::State& state)
{
    m_state = state;
    return true;
}

const FloatingBaseSystemAccelerationKinematics::State&
FloatingBaseSystemAccelerationKinematics::getState() const
{
    return m_state;
}

bool FloatingBaseSystemAccelerationKinematics::setControlInput(const Input& controlInput)
{
    m_controlInput = controlInput;
    return true;
}
