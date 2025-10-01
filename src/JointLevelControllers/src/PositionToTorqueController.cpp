/**
 * @file PositionToTorqueController.cpp
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/JointLevelControllers/PositionToTorqueController.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <limits>

using namespace BipedalLocomotion::JointLevelControllers;

struct PositionToTorqueController::Impl
{
    Eigen::VectorXd output; /**< Output current for each joint */
    PositionToJointControllerInput input; /**< Input data for the controller */
    bool outputValid{false}; /**< Flag indicating if the output is valid */

    Eigen::VectorXd kp; /**< Proportional gain for each joint */
    Eigen::VectorXd kd; /**< Derivative gain for each joint */

    /* optional parameters ------------------------------------------ */
    Eigen::VectorXd torqueLimit; /**< torque limit for each joint */

    Eigen::VectorXd positionError; /**< Position error vector */
};

PositionToTorqueController::PositionToTorqueController()
    : m_pimpl(std::make_unique<Impl>())
{
}

PositionToTorqueController::~PositionToTorqueController() = default;

bool PositionToTorqueController::advance()
{
    m_pimpl->outputValid = false;

    // Compute position error once and reuse
    m_pimpl->positionError = m_pimpl->input.referencePosition - m_pimpl->input.feedbackPosition;

    // Compute proportional term
    m_pimpl->output = m_pimpl->kp.cwiseProduct(m_pimpl->positionError);
    // Add derivative term
    m_pimpl->output -= m_pimpl->kd.cwiseProduct(m_pimpl->input.feedbackVelocity);

    // Clamp
    m_pimpl->output = //
        m_pimpl->output.cwiseMin(m_pimpl->torqueLimit).cwiseMax(-m_pimpl->torqueLimit);

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

bool PositionToTorqueController::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[PositionToTorqueController::initialize] ";
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

    // required parameters
    constexpr auto required = true;
    if (!loadGroupVector(ptr, "kp", joints, m_pimpl->kp, required)
        || !loadGroupVector(ptr, "kd", joints, m_pimpl->kd, required))
    {
        log()->error("{} Missing required parameters.", logPrefix);
        return false;
    }

    // check the size of the required parameters
    if (m_pimpl->kp.size() != joints.size() || m_pimpl->kd.size() != joints.size())
    {
        log()->error("{} Invalid size of the required parameters. All the required parameters "
                     "must have the same size of 'joints_list' (={}). Kp size = {}, Kd size = {}",
                     logPrefix,
                     joints.size(),
                     m_pimpl->kp.size(),
                     m_pimpl->kd.size());
        return false;
    }

    constexpr auto optional = false;
    const double inf = std::numeric_limits<double>::infinity();
    loadGroupVector(ptr, "torque_limit", joints, m_pimpl->torqueLimit, optional, inf);
    if (m_pimpl->torqueLimit.minCoeff() < 0.0)
    {
        log()->error("{} Invalid 'torque_limit'. All values must be non-negative.", logPrefix);
        return false;
    }

    m_pimpl->positionError.resize(joints.size());

    return true;
}

bool PositionToTorqueController::setInput(const PositionToJointControllerInput& input)
{
    constexpr auto logPrefix = "[PositionToTorqueController::setInput] ";
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
Eigen::VectorXd& PositionToTorqueController::getOutput() const
{
    return m_pimpl->output;
}

bool PositionToTorqueController::isOutputValid() const
{
    return m_pimpl->outputValid;
}
