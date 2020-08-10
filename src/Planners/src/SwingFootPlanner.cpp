/**
 * @file SwingFootPlanner.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Planners/SwingFootPlanner.h>

using namespace BipedalLocomotion::Planners;


using Vector1d = Eigen::Matrix<double,1,1>;

bool SwingFootPlanner::initialize(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    if (handler == nullptr)
    {
        std::cerr << "[SwingFootPlanner::initialize] The handler is not correctly initialized.";
        return false;
    }

    if (!handler->getParameter("sampling_time", m_dT))
    {
        std::cerr << "[SwingFootPlanner::initialize] Unable to initialize the sampling time for "
                     "the SwingFootPlanner.";
        return false;
    }

    if (!handler->getParameter("step_height", m_stepHeight))
    {
        std::cerr << "[SwingFootPlanner::initialize] Unable to initialize the step height for "
                     "the SwingFootPlanner.";
        return false;
    }

    if (!handler->getParameter("foot_apex_time", m_footApexTime))
    {
        std::cerr << "[SwingFootPlanner::initialize] Unable to initialize the foot apex time for "
                     "the SwingFootPlanner.";
        return false;
    }

    double footLandingVelocity;
    if (!handler->getParameter("foot_landing_velocity", footLandingVelocity))
    {
        std::cerr << "[SwingFootPlanner::initialize] Unable to initialize the foot landing "
                     "velocity for the SwingFootPlanner.";
        return false;
    }

    double footLandingAcceleration;
    if (!handler->getParameter("foot_landing_acceleration", footLandingAcceleration))
    {
        std::cerr << "[SwingFootPlanner::initialize] Unable to initialize the foot landing "
                     "acceleration for the SwingFootPlanner.";
        return false;
    }

    // check the parameters passed to the planner
    bool ok = (m_dT > 0) && ((m_footApexTime > 0) && (m_footApexTime < 1));
    if (!ok)
    {
        std::cerr << "[SwingFootPlanner::initialize] There is a problem in the parameters used to "
                     "configure the planner. Please remember to set a strictly positive "
                     "sampling_time, and a foot_apex_time strictly between 0 and 1."
                  << std::endl;
        return false;
    }

    m_planarPlanner.setInitialConditions(Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());
    m_planarPlanner.setFinalConditions(Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());


    m_heightPlanner.setInitialConditions(Vector1d::Zero(), Vector1d::Zero());
    m_heightPlanner.setFinalConditions(Vector1d::Constant(footLandingVelocity), Vector1d::Constant(footLandingAcceleration));

    return true;
}

void SwingFootPlanner::setContactList(const ContactList& contactList)
{
    // reset the time
    m_currentTrajectoryTime = 0;

    m_contactList = contactList;

    // set the first contact
    m_currentContactPtr = m_contactList.firstContact();
    m_state.transform = m_currentContactPtr->pose;
    m_state.spatialVelocity.setZero();
    m_state.spatialAcceleration.setZero();
    m_state.isInContact = true;

}

bool SwingFootPlanner::isValid() const
{
    return m_contactList.size() != 0;
}

const SwingFootPlannerState& SwingFootPlanner::get() const
{
    return m_state;
}

bool SwingFootPlanner::updateSE3Traj()
{
    // compute the trajectory at the current time
    const double shiftedTime = m_currentTrajectoryTime - m_currentContactPtr->deactivationTime;

    manif::SO3d rotation;
    auto angularVelocity(m_state.spatialVelocity.asSO3());
    auto angularAcceleration(m_state.spatialAcceleration.asSO3());
    if (!m_SO3Planner.evaluatePoint(shiftedTime,
                                    rotation,
                                    angularVelocity,
                                    angularAcceleration))
    {
        std::cerr << "[SwingFootPlanner::advance] Unable to get the SO(3) trajectory at time t = "
                  << m_currentTrajectoryTime << "." << std::endl;
        return false;
    }

    manif::SE3d::Translation position;
    if (!m_planarPlanner.evaluatePoint(m_currentTrajectoryTime,
                                       position.head<2>(),
                                       m_state.spatialVelocity.coeffs().head<2>(),
                                       m_state.spatialAcceleration.coeffs().head<2>()))
    {
        std::cerr << "[SwingFootPlanner::advance] Unable to get the planar trajectory at time t = "
                  << m_currentTrajectoryTime << "." << std::endl;
        return false;
    }

    if (!m_heightPlanner.evaluatePoint(m_currentTrajectoryTime,
                                       position.segment<1>(2),
                                       m_state.spatialVelocity.coeffs().segment<1>(2),
                                       m_state.spatialAcceleration.coeffs().segment<1>(2)))
    {
        std::cerr << "[SwingFootPlanner::advance] Unable to get the height trajectory at time t = "
                  << m_currentTrajectoryTime << "." << std::endl;
        return false;
    }

    // set the transform
    m_state.transform = manif::SE3d(position, rotation);

    m_state.isInContact = false;

    return true;
}

bool SwingFootPlanner::advance()
{
    // update the time taking into account the limits
    m_currentTrajectoryTime
        = std::min(m_dT + m_currentTrajectoryTime, m_contactList.lastContact()->deactivationTime);

    // here there are several case
    // 1. the link was already in contact with the environment and now it is still in contact
    // 2. the link was in contact with the environment but now not (This cannot happen for the
    // latest contact phase)
    // 3. the link was not in contact with the environment and now it is still not in contact (This
    // cannot happen for the latest contact phase)
    // 4. the link was not in contact with the environment but now it is in contact (This  cannot
    // happen for the latest contact phase)

    // This is the case (1). In this case we do not have to update the state
    if (m_state.isInContact && m_currentTrajectoryTime <= m_currentContactPtr->deactivationTime)
    {
        return true;
    }

    // In theory this should never happen. This check is only for spotting possible bugs
    if (m_currentContactPtr == m_contactList.end())
    {
        std::cerr << "[SwingFootPlanner::advance] During the last phase the link should be in "
                     "contact with the environment."
                  << std::endl;
        return false;
    }

    // This is the case (2)
    if (m_state.isInContact && m_currentTrajectoryTime > m_currentContactPtr->deactivationTime)
    {
        // create a new trajectory in SE(3)
        const auto nextContactPtr = std::next(m_currentContactPtr, 1);
        const double T = nextContactPtr->activationTime - m_currentContactPtr->deactivationTime;

        if (!m_SO3Planner.setRotations(m_currentContactPtr->pose.asSO3(), nextContactPtr->pose.asSO3(), T))
        {
            std::cerr << "[SwingFootPlanner::advance] Unable to set the initial and final "
                         "rotations for the SO(3) planner."
                      << std::endl;
            return false;
        }

        if (!m_planarPlanner.setKnots({m_currentContactPtr->pose.translation().head<2>(),
                                       nextContactPtr->pose.translation().head<2>()},
                                      {m_currentContactPtr->deactivationTime,
                                       nextContactPtr->activationTime}))
        {
            std::cerr << "[SwingFootPlanner::advance] Unable to set the knots for the planar "
                         "planner."
                      << std::endl;
            return false;
        }

        const double footHeightViaPointPos = (m_currentContactPtr->pose.translation()(2) +
                                              nextContactPtr->pose.translation()(2)) / 2.0 + m_stepHeight;

        const double footHeightViaPointTime = m_footApexTime * T + m_currentContactPtr->deactivationTime;

        if (!m_heightPlanner.setKnots({m_currentContactPtr->pose.translation().tail<1>(),
                                       Vector1d::Constant(footHeightViaPointPos),
                                       nextContactPtr->pose.translation().tail<1>()},
                                      {m_currentContactPtr->deactivationTime,
                                       footHeightViaPointTime,
                                       nextContactPtr->activationTime}))
        {
            std::cerr << "[SwingFootPlanner::advance] Unable to set the knots for the height "
                         "planner."
                      << std::endl;
            return false;
        }

        if (!this->updateSE3Traj())
        {
            std::cerr << "[SwingFootPlanner::advance] Unable to update the SE(3) trajectory."
                      << std::endl;
            return false;
        }

        return true;
    }


    // This is the case (3)
    if (!m_state.isInContact && m_currentTrajectoryTime > m_currentContactPtr->deactivationTime
        && m_currentTrajectoryTime <= std::next(m_currentContactPtr, 1)->activationTime)
    {
        if (!this->updateSE3Traj())
        {
            std::cerr << "[SwingFootPlanner::advance] Unable to update the SE(3) trajectory."
                      << std::endl;
            return false;
        }


        return true;
    }

    // This is the case (4)
    if (!m_state.isInContact && m_currentTrajectoryTime > m_currentContactPtr->deactivationTime
        && m_currentTrajectoryTime > std::next(m_currentContactPtr, 1)->activationTime)
    {
        std::advance(m_currentContactPtr, 1);

        m_state.transform = m_currentContactPtr->pose;
        m_state.spatialVelocity.setZero();
        m_state.spatialAcceleration.setZero();
        m_state.isInContact = true;

        return true;
    }

    return true;
}
