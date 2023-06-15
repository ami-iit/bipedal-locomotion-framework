/**
 * @file LinearTimeInvariantSystem.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>
#include <BipedalLocomotion/GenericContainer/NamedTuple.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ContinuousDynamicalSystem;

bool LinearTimeInvariantSystem::setSystemMatrices(const Eigen::Ref<const Eigen::MatrixXd>& A,
                                                  const Eigen::Ref<const Eigen::MatrixXd>& B)
{
    constexpr auto errorPrefix = "[LinearTimeInvariantSystem::setSystemMatrices]";
    if (A.rows() != B.rows())
    {
        log()->error("{} A and B must have the same number of rows.", errorPrefix);
        return false;
    }

    if (A.rows() != A.cols())
    {
        log()->error("{} A must be a square matrix.", errorPrefix);
        return false;
    }

    m_A = A;
    m_B = B;
    m_isInitialized = true;

    return true;
}

bool LinearTimeInvariantSystem::dynamics(const std::chrono::nanoseconds& time, StateDerivative& stateDerivative)
{
    constexpr auto errorPrefix = "[LinearTimeInvariantSystem::dynamics]";

    if (!m_isInitialized)
    {
        log()->error("{} Please initialize the matrices. Call setSystemMatrices()", errorPrefix);
        return false;
    }

    using namespace BipedalLocomotion::GenericContainer::literals;

    const Eigen::VectorXd& x = m_state.get_from_hash<"x"_h>();
    Eigen::VectorXd& dx = stateDerivative.get_from_hash<"dx"_h>();
    const Eigen::VectorXd& u = m_controlInput.get_from_hash<"u"_h>();

    if (x.size() != m_A.rows())
    {
        log()->error("{} The size of the vector 'state' is not coherent with the system matrices.",
                     errorPrefix);
        return false;
    }

    if (u.size() != m_B.cols())
    {
        log()->error("{} The size of the vector 'control input' is not coherent with the system "
                     "matrices.",
                     errorPrefix);
        return false;
    }

    dx.noalias() = m_A * x;
    dx.noalias() += m_B * u;

    return true;
}

bool LinearTimeInvariantSystem::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    return true;
}

bool LinearTimeInvariantSystem::setState(const LinearTimeInvariantSystem::State& state)
{
    m_state = state;
    return true;
}

const LinearTimeInvariantSystem::State& LinearTimeInvariantSystem::getState() const
{
    return m_state;
}

bool LinearTimeInvariantSystem::setControlInput(const LinearTimeInvariantSystem::Input& controlInput)
{
    m_controlInput = controlInput;
    return true;
}
