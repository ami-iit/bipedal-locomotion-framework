/**
 * @file SwingFootPlanner.cpp
 * @authors Giulio Romualdi
 * @copyright 2020, 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Planners/CubicSpline.h>
#include <BipedalLocomotion/Planners/QuinticSpline.h>
#include <BipedalLocomotion/Planners/SwingFootPlanner.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::Planners;
using namespace BipedalLocomotion::Contacts;

using Vector1d = Eigen::Matrix<double, 1, 1>;

bool SwingFootPlanner::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[SwingFootPlanner::initialize]";
    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} The handler is not correctly initialized.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("sampling_time", m_dT))
    {
        log()->error("{} Unable to initialize the sampling time.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("step_height", m_stepHeight))
    {
        log()->error("{} Unable to initialize the step height.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("foot_apex_time", m_footApexTime))
    {
        log()->error("{} Unable to initialize the foot apex time.", logPrefix);
        return false;
    }

    double footTakeOffVelocity = 0.0;
    if (!ptr->getParameter("foot_take_off_velocity", footTakeOffVelocity))
    {
        log()->info("{} Using default foot_take_off_velocity={}.", logPrefix, footTakeOffVelocity);
    }

    double footTakeOffAcceleration = 0.0;
    if (!ptr->getParameter("foot_take_off_acceleration", footTakeOffAcceleration))
    {
        log()->info("{} Using default foot_take_off_acceleration={}.",
                    logPrefix,
                    footTakeOffAcceleration);
    }

    double footLandingVelocity = 0.0;
    if (!ptr->getParameter("foot_landing_velocity", footLandingVelocity))
    {
        log()->info("{} Using default foot_landing_velocity={}.", logPrefix, footLandingVelocity);
    }

    double footLandingAcceleration = 0.0;
    if (!ptr->getParameter("foot_landing_acceleration", footLandingAcceleration))
    {
        log()->info("{} Using default foot_landing_acceleration={}.",
                    logPrefix,
                    footLandingAcceleration);
    }

    // check the parameters passed to the planner
    const bool ok = (m_dT > 0) && ((m_footApexTime > 0) && (m_footApexTime < 1));
    if (!ok)
    {
        log()->error("{} The sampling time should be a positive number and the foot_apex_time must "
                     "be strictly between 0 and 1.",
                     logPrefix);
        return false;
    }

    std::string interpolationMethod{"min_acceleration"};
    if (ptr->getParameter("interpolation_method", interpolationMethod))
    {
        if (interpolationMethod == "min_acceleration")
        {
            m_planarPlanner = std::make_unique<CubicSpline>();
            m_heightPlanner = std::make_unique<CubicSpline>();
        } else if (interpolationMethod == "min_jerk")
        {
            m_planarPlanner = std::make_unique<QuinticSpline>();
            m_heightPlanner = std::make_unique<QuinticSpline>();
        } else
        {
            log()->warn("{} The parameter named 'interpolation_method' must be equal to "
                        "'min_acceleration' or 'min_jerk'. The 'min_acceleration' method will be "
                        "used.",
                        logPrefix);
            m_planarPlanner = std::make_unique<CubicSpline>();
            m_heightPlanner = std::make_unique<CubicSpline>();
        }
    } else
    {
        log()->info("{} The parameter named 'interpolation_method' not found. The "
                    "'min_acceleration' method will be used.",
                    logPrefix);
        m_planarPlanner = std::make_unique<CubicSpline>();
        m_heightPlanner = std::make_unique<CubicSpline>();
    }

    m_planarPlanner->setInitialConditions(Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());
    m_planarPlanner->setFinalConditions(Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());

    m_heightPlanner->setInitialConditions(Vector1d::Constant(footTakeOffVelocity),
                                          Vector1d::Constant(footTakeOffAcceleration));
    m_heightPlanner->setFinalConditions(Vector1d::Constant(footLandingVelocity),
                                        Vector1d::Constant(footLandingAcceleration));

    return true;
}

void SwingFootPlanner::setContactListWithotResettingTheInternalTime(const ContactList& contactList)
{
    m_contactList = contactList;

    // set the first contact
    m_currentContactPtr = m_contactList.getPresentContact(m_currentTrajectoryTime);
    m_state.transform = m_currentContactPtr->pose;
    // m_state.mixedVelocity.setZero();
    // m_state.mixedAcceleration.setZero();
    // m_state.isInContact = true;
}


void SwingFootPlanner::setContactList(const ContactList& contactList)
{
    // reset the time
    m_currentTrajectoryTime = 0;

    m_contactList = contactList;

    // set the first contact
    m_currentContactPtr = m_contactList.firstContact();
    m_state.transform = m_currentContactPtr->pose;
    m_state.mixedVelocity.setZero();
    m_state.mixedAcceleration.setZero();
    m_state.isInContact = true;
}

bool SwingFootPlanner::isOutputValid() const
{
    return m_contactList.size() != 0;
}

const SwingFootPlannerState& SwingFootPlanner::getOutput() const
{
    return m_state;
}

bool SwingFootPlanner::updateSE3Traj()
{
    constexpr auto logPrefix = "[SwingFootPlanner::updateSE3Traj]";

    // compute the trajectory at the current time
    const double shiftedTime = m_currentTrajectoryTime - m_currentContactPtr->deactivationTime;

    manif::SO3d rotation;

    // note here we assume that the velocity is expressed using the mixed representation.
    // for this reason the velocity can be computed splitting the linear and the angular problem.
    // For the angular part we are interested in the angular velocity expressed in the inertial
    // frame (right trivialized velocity). For the liner part we are interested on the derivative of
    // the position vector w.r.t the time. A similar consideration can be done also for the
    // acceleration.

    // Note that here we use asSO3() to access to the angular velocity stored in the 6d-vector. (A
    // similar consideration can be done also for the acceleration.)
    auto angularVelocity(m_state.mixedVelocity.asSO3());
    auto angularAcceleration(m_state.mixedAcceleration.asSO3());
    if (!m_SO3Planner.evaluatePoint(shiftedTime, rotation, angularVelocity, angularAcceleration))
    {
        log()->error("{} Unable to get the SO(3) trajectory at time t = {}.",
                     logPrefix,
                     m_currentTrajectoryTime);
        return false;
    }

    manif::SE3d::Translation position;
    if (!m_planarPlanner->evaluatePoint(m_currentTrajectoryTime,
                                        position.head<2>(),
                                        m_state.mixedVelocity.coeffs().head<2>(),
                                        m_state.mixedAcceleration.coeffs().head<2>()))
    {
        log()->error("{} Unable to get the planar trajectory at time t = {}.",
                     logPrefix,
                     m_currentTrajectoryTime);
        return false;
    }

    if (!m_heightPlanner->evaluatePoint(m_currentTrajectoryTime,
                                        position.segment<1>(2),
                                        m_state.mixedVelocity.coeffs().segment<1>(2),
                                        m_state.mixedAcceleration.coeffs().segment<1>(2)))
    {
        log()->error("{} Unable to the height trajectory at time t = {}.",
                     logPrefix,
                     m_currentTrajectoryTime);
        return false;
    }

    // set the transform
    m_state.transform = manif::SE3d(position, rotation);

    m_state.isInContact = false;

    return true;
}

bool SwingFootPlanner::advance()
{
    constexpr auto logPrefix = "[SwingFootPlanner::advance]";

    // update the time taking into account the limits
    m_currentTrajectoryTime = std::min(m_dT + m_currentTrajectoryTime, //
                                       m_contactList.lastContact()->deactivationTime);

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
        log()->error("{} During the last phase the link should be in contact with the environment.",
                     logPrefix);
        return false;
    }

    // This is the case (2)
    if (m_state.isInContact && m_currentTrajectoryTime > m_currentContactPtr->deactivationTime)
    {
        // create a new trajectory in SE(3)
        const auto nextContactPtr = std::next(m_currentContactPtr, 1);
        const double T = nextContactPtr->activationTime - m_currentContactPtr->deactivationTime;

        if (!m_SO3Planner.setRotations(m_currentContactPtr->pose.asSO3(),
                                       nextContactPtr->pose.asSO3(),
                                       T))
        {
            log()->error("{} Unable to set the initial and final rotations for the SO(3) planner.",
                         logPrefix);
            return false;
        }

        if (!m_planarPlanner->setKnots({m_currentContactPtr->pose.translation().head<2>(),
                                        nextContactPtr->pose.translation().head<2>()},
                                       {m_currentContactPtr->deactivationTime,
                                        nextContactPtr->activationTime}))
        {
            log()->error("{} Unable to set the knots for the planar planner.", logPrefix);
            return false;
        }

        const double footHeightViaPointPos = (m_currentContactPtr->pose.translation()(2) //
                                              + nextContactPtr->pose.translation()(2))
                                                 / 2.0
                                             + m_stepHeight;

        const double footHeightViaPointTime = m_footApexTime * T //
                                              + m_currentContactPtr->deactivationTime;

        if (!m_heightPlanner->setKnots({m_currentContactPtr->pose.translation().tail<1>(),
                                        Vector1d::Constant(footHeightViaPointPos),
                                        nextContactPtr->pose.translation().tail<1>()},
                                       {m_currentContactPtr->deactivationTime,
                                        footHeightViaPointTime,
                                        nextContactPtr->activationTime}))
        {
            log()->error("{} Unable to set the knots for the knots for the height planner.",
                         logPrefix);
            return false;
        }

        if (!this->updateSE3Traj())
        {
            log()->error("{} Unable to update the SE(3) trajectory.", logPrefix);
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
            log()->error("{} Unable to update the SE(3) trajectory.", logPrefix);
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
        m_state.mixedVelocity.setZero();
        m_state.mixedAcceleration.setZero();
        m_state.isInContact = true;

        return true;
    }

    return true;
}
