/**
 * @file SwingFootPlanner.cpp
 * @authors Giulio Romualdi
 * @copyright 2020, 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Planners/CubicSpline.h>
#include <BipedalLocomotion/Planners/QuinticSpline.h>
#include <BipedalLocomotion/Planners/SwingFootPlanner.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <chrono>
#include <iterator>

using namespace BipedalLocomotion::Contacts;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Planners;

using Vector1d = Eigen::Matrix<double, 1, 1>;

bool SwingFootPlanner::initialize(std::weak_ptr<const IParametersHandler> handler)
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

    if (!ptr->getParameter("foot_take_off_velocity", m_footTakeOffVelocity))
    {
        log()->info("{} Using default foot_take_off_velocity={} m/s.",
                    logPrefix,
                    m_footTakeOffVelocity);
    }

    if (!ptr->getParameter("foot_take_off_acceleration", m_footTakeOffAcceleration))
    {
        log()->info("{} Using default foot_take_off_acceleration={} m/s^2.",
                    logPrefix,
                    m_footTakeOffAcceleration);
    }

    if (!ptr->getParameter("foot_landing_velocity", m_footLandingVelocity))
    {
        log()->info("{} Using default foot_landing_velocity={} m/s.",
                    logPrefix,
                    m_footLandingVelocity);
    }

    if (!ptr->getParameter("foot_landing_acceleration", m_footLandingAcceleration))
    {
        log()->info("{} Using default foot_landing_acceleration={} m/s^2.",
                    logPrefix,
                    m_footLandingAcceleration);
    }

    // check the parameters passed to the planner
    const bool ok = (m_dT > std::chrono::nanoseconds::zero())
                    && ((m_footApexTime > 0) && (m_footApexTime < 1));
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

    return true;
}

bool SwingFootPlanner::setContactList(const ContactList& contactList)
{
    constexpr auto logPrefix = "[SwingFootPlanner::setContactList]";

    if (contactList.size() == 0)
    {
        log()->error("{} The provided contact list is empty.", logPrefix);
        return false;
    }

    // if m_contactList is empty, then it is the first time the contact list is added to the planner
    // so we can:
    // 1. take it
    // 2. Set the current trajectory time  equal to the activationTime of the first element in the
    // contact list
    if (m_contactList.size() == 0)
    {
        m_contactList = contactList;
        m_currentContactPtr = m_contactList.firstContact();

        // we set the time just an instant before the first activation time. In order to make the
        // block running we need to call advance at least one.
        this->setTime(m_currentContactPtr->activationTime - m_dT);
        m_state.transform = m_currentContactPtr->pose;
        m_state.mixedVelocity.setZero();
        m_state.mixedAcceleration.setZero();
        m_state.isInContact = true;

        return true;
    }

    // in this case the contact list is not empty. So we should check if it is possible to update
    // the list. Given some limitation of the framework (mainly due to the SO3 trajectory
    // generation) for the time being we support only the following case:
    // - Given the current time instant the both the original and the new contact list have an
    //   active contact at the same position.
    // - If the contact is not active we have to check that:
    //    1. the final orientation may change still the error (in the tangent space) between the
    //       new orientation and the current one should be parallel to the current velocity and
    //       acceleration vectors. This is required to keep the SO3Planner problem still treatable
    //       online. This check is not done here since the SO3Planner will complain in case of
    //       issues.
    //    2. the swing foot trajectory duration did not change

    // Here the case where the contact is active

    // get the contact ative at the given time
    auto newContactAtCurrentTime = contactList.getPresentContact(m_currentTrajectoryTime);

    // if the contact is active and the new contact list has an active contact at the give time
    // instant with the same position and orientation
    if (m_state.isInContact && newContactAtCurrentTime != contactList.cend()
        && newContactAtCurrentTime->isContactActive(m_currentTrajectoryTime)
        && (*newContactAtCurrentTime) == (*m_currentContactPtr))
    {
        m_contactList = contactList;
        m_currentContactPtr = m_contactList.getPresentContact(m_currentTrajectoryTime);

        return true;
    }

    // if the contact is active and the new contact list has an active contact at the give time
    // instant
    if (!m_state.isInContact && newContactAtCurrentTime != contactList.cend()
        && newContactAtCurrentTime != contactList.lastContact())
    {
        // we take the next contact of the new contact list and we check:
        // 1. the final orientation may change still the error (in the tangent space) between the
        //     new orientation and the current one should be parallel to the current velocity and
        //     acceleration vectors. This is required to keep the SO3Planner problem still treatable
        //     online. This check is not done here since the SO3Planner will complain in case of
        //     issues.
        //
        // 2. the swing foot trajectory duration didn't change
        auto newNextContactAtCurrentTime = std::next(newContactAtCurrentTime, 1);

        // we are sure that this will exist since we are in swing phase
        auto nextContactAtCurrentTime = std::next(m_currentContactPtr, 1);
        assert(nextContactAtCurrentTime != m_contactList.cend());

        // evaluate the duration of the two swing foot trajectories
        const auto originalSwingFootTrajectoryDuration = nextContactAtCurrentTime->activationTime //
                                                         - m_currentContactPtr->deactivationTime;
        const auto newSwingFootTrajectoryDuration = newNextContactAtCurrentTime->activationTime
                                                    - newContactAtCurrentTime->deactivationTime;

        if (newSwingFootTrajectoryDuration != originalSwingFootTrajectoryDuration)
        {
            log()->error("{} Failing to update the contact list. At the give time instant t = {} "
                         "the foot is swinging. In order to update the conatct list we ask. The "
                         "duration of the swing foot phase do not change. In this case we have. "
                         "New Orientation {}, original orientation {}. New duration {}, original "
                         "duration {}.",
                         logPrefix,
                         m_currentTrajectoryTime,
                         newNextContactAtCurrentTime->pose.quat().coeffs().transpose(),
                         nextContactAtCurrentTime->pose.quat().coeffs().transpose(),
                         newSwingFootTrajectoryDuration,
                         originalSwingFootTrajectoryDuration);
            return false;
        }

        // update the contact list and the pointer
        m_contactList = contactList;
        m_currentContactPtr = m_contactList.getPresentContact(m_currentTrajectoryTime);
        if (!this->createSE3Traj(m_state.mixedVelocity.lin().head<2>(),
                                 m_state.mixedAcceleration.lin().head<2>(),
                                 m_state.mixedVelocity.lin().tail<1>(),
                                 m_state.mixedAcceleration.lin().tail<1>(),
                                 m_state.mixedVelocity.ang(),
                                 m_state.mixedAcceleration.ang()))
        {
            log()->error("{} Unable to create the new SE(3) trajectory.", logPrefix);
            return false;
        }

        if (!this->updateSE3Traj())
        {
            log()->error("{} Unable to update the SE(3) trajectory.", logPrefix);
            return false;
        }

        return true;
    }

    log()->error("{} Unable to set the contact phase list.", logPrefix);
    return false;
}

bool SwingFootPlanner::isOutputValid() const
{
    return m_isOutputValid;
}

const SwingFootPlannerState& SwingFootPlanner::getOutput() const
{
    return m_state;
}

void SwingFootPlanner::setTime(const std::chrono::nanoseconds& time)
{
    m_currentTrajectoryTime = time;
}

bool SwingFootPlanner::createSE3Traj(Eigen::Ref<const Eigen::Vector2d> initialPlanarVelocity,
                                     Eigen::Ref<const Eigen::Vector2d> initialPlanarAcceleration,
                                     Eigen::Ref<const Vector1d> initialVerticalVelocity,
                                     Eigen::Ref<const Vector1d> initialVerticalAcceleration,
                                     const manif::SO3d::Tangent& initialAngularVelocity,
                                     const manif::SO3d::Tangent& initialAngularAcceleration)
{
    constexpr auto logPrefix = "[SwingFootPlanner::createSE3Traj]";

    // create a new trajectory in SE(3)
    const auto nextContactPtr = std::next(m_currentContactPtr, 1);

    if (nextContactPtr == m_contactList.cend())
    {
        log()->error("{} Invalid next contact. Time {}.", logPrefix, m_currentTrajectoryTime);
        return false;
    }

    // The rotation cannot change when the contact list is updated. For this reason we can build the
    // trajectory considering the initial and the final conditions
    m_staringTimeOfCurrentSO3Traj = m_currentTrajectoryTime;
    const auto T = nextContactPtr->activationTime - m_staringTimeOfCurrentSO3Traj;
    m_SO3Planner.setInitialConditions(initialAngularVelocity, initialAngularAcceleration);
    m_SO3Planner.setFinalConditions(manif::SO3d::Tangent::Zero(), manif::SO3d::Tangent::Zero());
    if (!m_SO3Planner.setRotations(m_state.transform.quat(), nextContactPtr->pose.asSO3(), T))
    {
        log()->error("{} Unable to set the initial and final rotations for the SO(3) planner.",
                     logPrefix);
        return false;
    }

    m_planarPlanner->setInitialConditions(initialPlanarVelocity, initialPlanarAcceleration);
    m_planarPlanner->setFinalConditions(Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());

    // for the planar case we start at the current state at the given time
    if (!m_planarPlanner->setKnots({m_state.transform.translation().head<2>(),
                                    nextContactPtr->pose.translation().head<2>()},
                                   {m_currentTrajectoryTime, nextContactPtr->activationTime}))
    {
        log()->error("{} Unable to set the knots for the planar planner.", logPrefix);
        return false;
    }

    // The foot maximum height point is given by the highest point plus the offset
    // If the robot is walking on a plane with height equal to zero, the footHeightViaPointPos is
    // given by the stepHeight
    const double footHeightViaPointPos = std::max(m_currentContactPtr->pose.translation()(2),
                                                  nextContactPtr->pose.translation()(2))
                                         + m_stepHeight;

    // the cast is required since m_footApexTime is a floating point number between 0 and 1
    const std::chrono::nanoseconds footHeightViaPointTime
        = std::chrono::duration_cast<std::chrono::nanoseconds>(
            m_footApexTime * T + m_currentContactPtr->deactivationTime);

    m_heightPlanner->setInitialConditions(initialVerticalVelocity, initialVerticalAcceleration);
    m_heightPlanner->setFinalConditions(Vector1d::Constant(m_footLandingVelocity),
                                        Vector1d::Constant(m_footLandingAcceleration));

    if (m_currentTrajectoryTime < footHeightViaPointTime)
    {
        if (!m_heightPlanner->setKnots({m_state.transform.translation().tail<1>(),
                                        Vector1d::Constant(footHeightViaPointPos),
                                        nextContactPtr->pose.translation().tail<1>()},
                                       {m_currentTrajectoryTime,
                                        footHeightViaPointTime,
                                        nextContactPtr->activationTime}))
        {
            log()->error("{} Unable to set the knots for the knots for the height planner.",
                         logPrefix);
            return false;
        }
    } else
    {
        if (!m_heightPlanner->setKnots({m_state.transform.translation().tail<1>(),
                                        nextContactPtr->pose.translation().tail<1>()},
                                       {m_currentTrajectoryTime, nextContactPtr->activationTime}))
        {
            log()->error("{} Unable to set the knots for the knots for the height planner.",
                         logPrefix);
            return false;
        }
    }

    return true;
}

bool SwingFootPlanner::updateSE3Traj()
{
    constexpr auto logPrefix = "[SwingFootPlanner::updateSE3Traj]";

    // compute the trajectory at the current time
    const auto shiftedTime = m_currentTrajectoryTime - m_staringTimeOfCurrentSO3Traj;

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

    m_isOutputValid = false;

    if (m_contactList.size() == 0)
    {
        log()->error("{} Empty contact list. Please call SwingFootPlanner::setContactList()",
                     logPrefix);
        return false;
    }

    // update the internal time
    m_currentTrajectoryTime += m_dT;

    // here there are several case
    // 0. the current time exceed the last deactivation time. This means that the trajectory ended
    // in this case lets assume that the output is valid and we avoid to udate the state
    // 1. the link was already in contact with the environment and now it is still in contact
    // 2. the link was in contact with the environment but now not (This cannot happen for the
    // latest contact phase)
    // 3. the link was not in contact with the environment and now it is still not in contact (This
    // cannot happen for the latest contact phase)
    // 4. the link was not in contact with the environment but now it is in contact (This  cannot
    // happen for the latest contact phase)

    // This is case (0). In this case we do not have to update the state
    if (m_state.isInContact
        && m_currentTrajectoryTime >= m_contactList.lastContact()->deactivationTime)
    {
        log()->debug("{} The time stored in the planner is greater equal than the last "
                     "deactivation time. This is currently handled by this block but it may crate "
                     "issues. Last deactivation time: {}, Inner time {}.",
                     logPrefix,
                     std::chrono::duration<double>(m_contactList.lastContact()->deactivationTime),
                     std::chrono::duration<double>(m_currentTrajectoryTime));
        m_isOutputValid = true;
        return true;
    }

    // This is the case (1). In this case we do not have to update the state
    if (m_state.isInContact && m_currentTrajectoryTime < m_currentContactPtr->deactivationTime)
    {
        m_isOutputValid = true;
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
    if (m_state.isInContact && m_currentTrajectoryTime >= m_currentContactPtr->deactivationTime)
    {
        if (!this->createSE3Traj(Eigen::Vector2d::Zero(),
                                 Eigen::Vector2d::Zero(),
                                 Vector1d::Constant(m_footTakeOffVelocity),
                                 Vector1d::Constant(m_footTakeOffAcceleration),
                                 manif::SO3d::Tangent::Zero(),
                                 manif::SO3d::Tangent::Zero()))
        {
            log()->error("{} Unable to create the new SE(3) trajectory.", logPrefix);
            return false;
        }

        if (!this->updateSE3Traj())
        {
            log()->error("{} Unable to update the SE(3) trajectory.", logPrefix);
            return false;
        }

        m_isOutputValid = true;
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

        m_isOutputValid = true;
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

        m_isOutputValid = true;
        return true;
    }

    log()->error("{} Unable to evaluate the next step in the swing foot planner. This should never "
                 "happen.",
                 logPrefix);
    return false;
}
