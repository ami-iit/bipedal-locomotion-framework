/**
 * @file PositionToCurrentController.cpp
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/JointLevelControllers/PositionToCurrentController.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <limits>

using namespace BipedalLocomotion::JointLevelControllers;

struct PositionToCurrentController::Impl
{
    Eigen::VectorXd output; /**< Output current for each joint */
    PositionToCurrentControllerInput input; /**< Input data for the controller */
    bool outputValid{false}; /**< Flag indicating if the output is valid */

    Eigen::VectorXd kp; /**< Proportional gain for each joint */
    Eigen::VectorXd kd; /**< Derivative gain for each joint */
    Eigen::VectorXd gearRatio; /**< Gear ratio for each joint */
    Eigen::VectorXd kTau; /**< Torque constant for each joint */

    Eigen::VectorXd torqueToCurrentFactor; /**< Precomputed 1/(gearRatio * kTau) */

    /* optional parameters ------------------------------------------ */
    Eigen::VectorXd currentLimit; /**< Current limit for each joint */
    Eigen::VectorXd coulombFriction; /**< Coulomb friction for each joint */
    Eigen::VectorXd activationVelocity; /**< Friction activation velocity [rad/s] */

    /* TN-curve coefficients & knee -------------------------------- */
    Eigen::VectorXd ratedSpeed; /**< ω_rated */
    Eigen::VectorXd noLoadSpeed; /**< ω_0 */
    Eigen::VectorXd satSlope; /**< m */
    Eigen::VectorXd satIntercept; /**< b */

    Eigen::VectorXd dynamicLimit; /**< Dynamic limit for each joint */

    Eigen::VectorXd positionError; /**< Position error vector */

    double currentSafetyFactor{1.0}; /**< Safety factor for current limits */
};

PositionToCurrentController::PositionToCurrentController()
    : m_pimpl(std::make_unique<Impl>())
{
}

PositionToCurrentController::~PositionToCurrentController() = default;

bool PositionToCurrentController::advance()
{
    m_pimpl->outputValid = false;

    // Compute position error once and reuse
    m_pimpl->positionError = m_pimpl->input.referencePosition - m_pimpl->input.feedbackPosition;

    // Compute proportional term
    m_pimpl->output = m_pimpl->kp.cwiseProduct(m_pimpl->positionError);
    // Add derivative term
    m_pimpl->output -= m_pimpl->kd.cwiseProduct(m_pimpl->input.feedbackVelocity);

    // Add friction term efficiently: tau_friction = coulomb_friction * tanh(activation_velocity *
    // feedback_velocity) Compute friction contribution directly to avoid intermediate temporaries
    for (std::size_t j = 0; j < m_pimpl->output.size(); ++j)
    {
        const double tanhArg = m_pimpl->input.feedbackVelocity[j] / m_pimpl->activationVelocity[j];
        m_pimpl->output[j] += m_pimpl->coulombFriction[j] * std::tanh(tanhArg);
    }

    // Convert torque to current efficiently using precomputed factor
    m_pimpl->output = m_pimpl->output.cwiseProduct(m_pimpl->torqueToCurrentFactor);

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
        // Apply safety factor to dynamic limit
        m_pimpl->dynamicLimit[j] *= m_pimpl->currentSafetyFactor;
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
        if (isRequired)
        {
            BipedalLocomotion::log()->error("[loadGroupVector] Unable to find the parameter named "
                                            "{}",
                                            name);
        }
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
        || !loadGroupVector(ptr, "kd", joints, m_pimpl->kd, required)
        || !loadGroupVector(ptr, "gearbox", joints, m_pimpl->gearRatio, required)
        || !loadGroupVector(ptr, "k_tau", joints, m_pimpl->kTau, required))
    {
        log()->error("{} Missing required parameters.", logPrefix);
        return false;
    }

    // check the size of the required parameters
    if (m_pimpl->kp.size() != joints.size() || m_pimpl->kd.size() != joints.size()
        || m_pimpl->gearRatio.size() != joints.size() || m_pimpl->kTau.size() != joints.size())
    {
        log()->error("{} Invalid size of the required parameters. All the required parameters "
                     "must have the same size of 'joints_list' (={}). Kp size = {}, Kd size = {}, "
                     "GearRatio size = {}, KTau size = {}",
                     logPrefix,
                     joints.size(),
                     m_pimpl->kp.size(),
                     m_pimpl->kd.size(),
                     m_pimpl->gearRatio.size(),
                     m_pimpl->kTau.size());
        return false;
    }

    // Precompute torque-to-current conversion factor for efficiency
    m_pimpl->torqueToCurrentFactor
        = (m_pimpl->gearRatio.cwiseProduct(m_pimpl->kTau)).cwiseInverse();

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
    loadGroupVector(ptr, "activation_velocity", joints, m_pimpl->activationVelocity, optional, 1e-5);

    /* TN knee & zero-current speed -------------------------------- */
    loadGroupVector(ptr, "rated_speed", joints, m_pimpl->ratedSpeed, optional, inf);
    loadGroupVector(ptr, "no_load_speed", joints, m_pimpl->noLoadSpeed, optional, inf);

    if (m_pimpl->noLoadSpeed.minCoeff() < 0.0)
    {
        log()->error("{} Invalid 'no_load_speed'. All values must be non-negative.", logPrefix);
        return false;
    }

    if (m_pimpl->ratedSpeed.minCoeff() < 0.0)
    {
        log()->error("{} Invalid 'rated_speed'. All values must be non-negative.", logPrefix);
        return false;
    }

    /* compute slope/intercept (or disable) ------------------------ */
    m_pimpl->satSlope.resize(joints.size());
    m_pimpl->satIntercept.resize(joints.size());
    m_pimpl->positionError.resize(joints.size());

    if (!ptr->getParameter("current_safety_factor", m_pimpl->currentSafetyFactor))
    {
        log()->warn("{} Parameter 'current_safety_factor' not found. Using default value 1.0.",
                    logPrefix);
        m_pimpl->currentSafetyFactor = 1.0;
    }

    // check if the safety factor is valid
    if (m_pimpl->currentSafetyFactor <= 0.0 || m_pimpl->currentSafetyFactor > 1.0)
    {
        log()->error("{} Invalid 'current_safety_factor'. Must be in the range (0, 1].", logPrefix);
        return false;
    }

    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        const bool finiteImax = std::isfinite(m_pimpl->currentLimit[i]);
        const bool finiteNoLoadSpeed = std::isfinite(m_pimpl->noLoadSpeed[i]);
        const bool finiteRatedSpead = std::isfinite(m_pimpl->ratedSpeed[i]);
        const bool validTN = finiteImax && finiteNoLoadSpeed && finiteRatedSpead
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
            m_pimpl->noLoadSpeed[i] = inf;

            log()->debug("{} Invalid TN curve for joint {}.", logPrefix, joints[i]);
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
