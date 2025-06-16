/**
 * @file PositionToCurrentController.cpp
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/RobotInterface/PositionToCurrentController.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <limits>

using namespace BipedalLocomotion::RobotInterface;

struct PositionToCurrentController::Impl
{
    Eigen::VectorXd output; /**< Output current for each joint */
    PositionToCurrentControllerInput input; /**< Input data for the controller */
    bool outputValid{false}; /**< Flag indicating if the output is valid */

    Eigen::VectorXd kp; /**< Proportional gain for each joint */
    Eigen::VectorXd gearRatio; /**< Gear ratio for each joint */
    Eigen::VectorXd kTau; /**< Torque constant for each joint */

    /* optional parameters ------------------------------------------ */
    Eigen::VectorXd currentLimit; /**< Current limit for each joint */
    Eigen::VectorXd coulombFriction; /**< Coulomb friction for each joint */

    /* TN-curve coefficients & knee -------------------------------- */
    Eigen::VectorXd ratedSpeed; /**< ω_rated */
    Eigen::VectorXd noLoadSpeed; /**< ω_0 */
    Eigen::VectorXd satSlope; /**< m */
    Eigen::VectorXd satIntercept; /**< b */

    Eigen::VectorXd dynamicLimit; /**< Dynamic limit for each joint */
};

PositionToCurrentController::PositionToCurrentController()
    : m_pimpl(std::make_unique<Impl>())
{
}

PositionToCurrentController::~PositionToCurrentController() = default;

bool PositionToCurrentController::advance()
{
    m_pimpl->outputValid = false;

    // raw proportional + friction term
    m_pimpl->output
        = m_pimpl->kp.cwiseProduct(m_pimpl->input.referencePosition
                                   - m_pimpl->input.feedbackPosition)
          + m_pimpl->coulombFriction.cwiseProduct(m_pimpl->input.feedbackVelocity.cwiseSign());

    // torque -> current
    m_pimpl->output = m_pimpl->output.cwiseQuotient(m_pimpl->gearRatio.cwiseProduct(m_pimpl->kTau));

    for (std::size_t j = 0; j < m_pimpl->dynamicLimit.size(); ++j)
    {
        const double absVel = std::abs(m_pimpl->input.feedbackVelocity[j]);

        if (absVel >= m_pimpl->noLoadSpeed[j])
        {
            m_pimpl->dynamicLimit[j] = 0.0; /* no current at no-load speed */
        } else if (absVel <= m_pimpl->ratedSpeed[j])
        { /* flat region or no TN data */
            m_pimpl->dynamicLimit[j] = m_pimpl->currentLimit[j];
        } else
        { /* linear fall-off */
            m_pimpl->dynamicLimit[j] = m_pimpl->satSlope[j] * absVel + m_pimpl->satIntercept[j];
        }
    }

    // Clamp
    m_pimpl->output
        = m_pimpl->output.cwiseMin(m_pimpl->dynamicLimit).cwiseMax(-m_pimpl->dynamicLimit);

    m_pimpl->outputValid = true;
    return true;
}

/* ------------------------------------------------------------------ */
static bool loadGroupVector(
    const std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>& ptr,
    const std::string& name,
    const std::vector<std::string>& joints,
    Eigen::VectorXd& out,
    bool isRequired,
    double defaultVal = 0.0)
{
    out.resize(joints.size());
    for (int i = 0; i < joints.size(); i++)
    {
        out[i] = defaultVal;
    }

    auto g = ptr->getGroup(name).lock();
    if (g == nullptr)
    {
        return !isRequired; // if the group is not found and it's not required, return true
    }

    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        if (!g->getParameter(joints[i], out[i]) && isRequired)
        {
            return false;
        }
    }
    return true;
}

bool PositionToCurrentController::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[PositionToCurrentController::initialize] ";
    const auto ptr = handler.lock();
    if (!ptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return false;
    }

    std::vector<std::string> joints;
    if (!ptr->getParameter("joints_list", joints))
    {
        log()->error("{} Missing 'joints_list'.", logPrefix);
        return false;
    }

    m_pimpl->dynamicLimit.resize(joints.size());

    // required parameters
    constexpr auto required = true;
    if (!loadGroupVector(ptr, "kp", joints, m_pimpl->kp, required)
        || !loadGroupVector(ptr, "gearbox", joints, m_pimpl->gearRatio, required)
        || !loadGroupVector(ptr, "k_tau", joints, m_pimpl->kTau, required))
    {
        log()->error("{} Missing required parameters.", logPrefix);
        return false;
    }

    // optional (flat) current limit & friction
    constexpr auto optional = false;
    const double inf = std::numeric_limits<double>::infinity();
    loadGroupVector(ptr, "current_limit", joints, m_pimpl->currentLimit, optional, inf);
    if (m_pimpl->currentLimit.minCoeff() < 0.0)
    {
        log()->error("{} Invalid 'current_limit'. All values must be non-negative.", logPrefix);
        return false;
    }

    loadGroupVector(ptr, "coulomb_friction", joints, m_pimpl->coulombFriction, optional, 0.0);

    /* TN knee & zero-current speed -------------------------------- */
    loadGroupVector(ptr, "rated_speed", joints, m_pimpl->ratedSpeed, optional, -1.0);
    loadGroupVector(ptr, "no_load_speed", joints, m_pimpl->noLoadSpeed, optional, -1.0);

    /* compute slope/intercept (or disable) ------------------------ */
    m_pimpl->satSlope.resize(joints.size());
    m_pimpl->satIntercept.resize(joints.size());

    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        const bool finiteImax = std::isfinite(m_pimpl->currentLimit[i]);
        const bool validTN = finiteImax && (m_pimpl->ratedSpeed[i] > 0.0)
                             && (m_pimpl->noLoadSpeed[i] > m_pimpl->ratedSpeed[i]);

        if (validTN)
        {
            // den = omega_0 - omega_rated
            const double denom = m_pimpl->noLoadSpeed[i] - m_pimpl->ratedSpeed[i];

            // Compute m and b of the linear function
            m_pimpl->satSlope[i] = -m_pimpl->currentLimit[i] / denom;
            m_pimpl->satIntercept[i]
                = m_pimpl->currentLimit[i] - m_pimpl->satSlope[i] * m_pimpl->ratedSpeed[i]; /* b */
        } else
        {
            // no TN curve
            m_pimpl->satSlope[i] = 0.0;
            m_pimpl->satIntercept[i] = m_pimpl->currentLimit[i];
            m_pimpl->ratedSpeed[i] = inf;
        }
    }

    return true;
}

bool PositionToCurrentController::setInput(const PositionToCurrentControllerInput& input)
{
    constexpr auto logPrefix = "[PositionToCurrentController::setInput] ";
    const auto sz = m_pimpl->kp.size();

    if (input.referencePosition.size() != sz || input.feedbackPosition.size() != sz
        || input.feedbackVelocity.size() != sz)
    {
        log()->error("{} Invalid input size. Expected {}. Reference: {}, Feedback position: {}, "
                     "Feedback velocity: {}",
                     logPrefix,
                     sz,
                     input.referencePosition.size(),
                     input.feedbackPosition.size(),
                     input.feedbackVelocity.size());
        return false;
    }

    m_pimpl->input = input;
    return true;
}

/* ------------------------------------------------------------------ */
Eigen::VectorXd& PositionToCurrentController::getOutput() const
{
    return m_pimpl->output;
}

bool PositionToCurrentController::isOutputValid() const
{
    return m_pimpl->outputValid;
}
