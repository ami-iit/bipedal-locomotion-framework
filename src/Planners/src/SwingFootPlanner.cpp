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

    // if m_contactList is empty or the output is not valid, then it is the first time the contact
    // list is added to the planner so we can:
    // 1. take it
    // 2. Set the current trajectory time equal to the activationTime of the first element in the
    // contact list
    if (m_contactList.size() == 0)
    {
        m_contactList = contactList;
        m_lastValidContact = *m_contactList.firstContact();

        // we set the time just an instant before the first activation time. In order to make the
        // block running we need to call advance at least one.
        this->setTime(m_lastValidContact.activationTime);
        m_state.transform = m_lastValidContact.pose;
        m_state.mixedVelocity.setZero();
        m_state.mixedAcceleration.setZero();
        m_state.isInContact = true;
        m_state.time = m_lastValidContact.activationTime;

        return true;
    }

    // in this case the contact list is not empty. So we should check if it is possible to update
    // the list. Given some limitation of the framework (mainly due to the SO3 trajectory
    // generation) for the time being we support only the following case:
    // - Given the current time instant the both the original and the new contact list have an
    //   active contact at the same position.
    // - If the contact is not active we have to check that:
    //    1. the final orientation may change, still the error (in the tangent space) between the
    //       new orientation and the current one should be parallel to the current velocity and
    //       acceleration vectors. This is required to keep the SO3Planner problem still treatable
    //       online. This check is not done here since the SO3Planner will complain in case of
    //       issues.
    //    2. the swing foot trajectory duration did not change

    // Here the case where the contact is active

    // get the contact active at the given time
    auto newContactAtCurrentTime = contactList.getPresentContact(m_currentTrajectoryTime);
    auto newNextContactAtCurrentTime = contactList.getNextContact(m_currentTrajectoryTime);
    auto currentContactAtCurrentTime = m_contactList.getPresentContact(m_currentTrajectoryTime);
    bool isCurrentContactActive
        = currentContactAtCurrentTime->isContactActive(m_currentTrajectoryTime);

    // if the contact is active and the new contact list has an active contact at the give time
    // instant that is the same we update the contact list since new contact in the future may be
    // added
    if (isCurrentContactActive && newContactAtCurrentTime != contactList.cend()
        && newContactAtCurrentTime->isContactActive(m_currentTrajectoryTime)
        && (*newContactAtCurrentTime) == (*currentContactAtCurrentTime))
    {
        // the list of the contact must be updated since a new contact in the future may appear
        m_contactList = contactList;

        // // TODO this line may not be needed
        // m_lastValidContact = *m_contactList.getPresentContact(m_state.time);
        // log()->warn("caso 1");

        return true;
    }

    // if the contact is active and the new contact list has an active contact at the give time
    // instant that is different we update the contact
    if (isCurrentContactActive && newContactAtCurrentTime != contactList.cend()
        && newContactAtCurrentTime->isContactActive(m_currentTrajectoryTime)
        && (*newContactAtCurrentTime) != (*currentContactAtCurrentTime))
    {
        // we need to check that the pose is the same
        // pose is an SE3 manif object so we cannot use the == operator
        // Read it as
        // log(m_currentContactPtr->pose.inverse() * newContactAtCurrentTime->pose).vee()
        constexpr double tolerance = 1e-6;
        if (!(newContactAtCurrentTime->pose - currentContactAtCurrentTime->pose)
                 .coeffs()
                 .isZero(tolerance))
        {
            log()->error("{} The pose of the contact active at the current time instant is "
                         "different from the one of the current contact. This is not supported. "
                         "Original contact: {}, new contact: {}. Error {}",
                         logPrefix,
                         currentContactAtCurrentTime->pose.coeffs().transpose(),
                         newContactAtCurrentTime->pose.coeffs().transpose(),
                         (newContactAtCurrentTime->pose - currentContactAtCurrentTime->pose)
                             .coeffs()
                             .transpose());
            return false;
        }

        // since the pose did not change we suppose that the only change is in the timing
        m_contactList = contactList;
        m_lastValidContact = *m_contactList.getPresentContact(m_state.time);
        log()->warn("caso 2 {}", m_lastValidContact.name);

        return true;
    }

    // if the contact is not active and the new contact list has an active contact at the give time
    // instant
    
    if (!isCurrentContactActive && newNextContactAtCurrentTime != contactList.cend())
    {
        // we take the next contact of the new contact list and we check:
        // 1. the final orientation may change still the error (in the tangent space) between the
        //     new orientation and the current one should be parallel to the current velocity and
        //     acceleration vectors. This is required to keep the SO3Planner problem still treatable
        //     online. This check is not done here since the SO3Planner will complain in case of
        //     issues.
        //
        // 2. the swing foot trajectory duration didn't change

        // we are sure that this will exist since we are in swing phase
        auto nextContactAtCurrentTime = m_contactList.getNextContact(m_currentTrajectoryTime);
        assert(nextContactAtCurrentTime != m_contactList.cend());

        // evaluate the duration of the two swing foot trajectories
        if (nextContactAtCurrentTime->activationTime != newNextContactAtCurrentTime->activationTime)
        {
            log()->error("{} Failing to update the contact list. At the give time instant t = {} "
                         "the foot is swinging. In order to update the contact list we ask. The "
                         "duration of the swing foot phase do not change. In this case we have. "
                         "New Orientation {}, original orientation {}. New activation {}, original "
                         "activation {}.",
                         logPrefix,
                         m_currentTrajectoryTime,
                         newNextContactAtCurrentTime->pose.quat().coeffs().transpose(),
                         nextContactAtCurrentTime->pose.quat().coeffs().transpose(),
                         newNextContactAtCurrentTime->activationTime,
                         nextContactAtCurrentTime->activationTime);
            return false;
        }


        m_contactList = contactList;

        // update the SE(3) trajectory only if the foot was not in contact at the previous time step
        if (!m_state.isInContact)
        {
            // // we need to update the SE(3) trajectory to attach the new trajectory to the current
            // // time evaluateSE3Traj is called before create to evaluate the initial point of the new
            // // trajectory
            // SwingFootPlannerState dummyState;
            // if (!this->evaluateSE3Traj(dummyState))
            // {
            //     log()->error("{} Unable to update the SE(3) trajectory.", logPrefix);
            //     return false;
            // }

            if (!this->createSE3Traj(m_state.transform,
                                     m_state.mixedVelocity.lin().head<2>(),
                                     m_state.mixedAcceleration.lin().head<2>(),
                                     m_state.mixedVelocity.lin().tail<1>(),
                                     m_state.mixedAcceleration.lin().tail<1>(),
                                     m_state.mixedVelocity.ang(),
                                     m_state.mixedAcceleration.ang()))
            {
                log()->error("{} Unable to create the new SE(3) trajectory.", logPrefix);
                return false;
            }
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

bool SwingFootPlanner::createSE3Traj(const manif::SE3d& initialPose,
                                     Eigen::Ref<const Eigen::Vector2d> initialPlanarVelocity,
                                     Eigen::Ref<const Eigen::Vector2d> initialPlanarAcceleration,
                                     Eigen::Ref<const Vector1d> initialVerticalVelocity,
                                     Eigen::Ref<const Vector1d> initialVerticalAcceleration,
                                     const manif::SO3d::Tangent& initialAngularVelocity,
                                     const manif::SO3d::Tangent& initialAngularAcceleration)
{
    constexpr auto logPrefix = "[SwingFootPlanner::createSE3Traj]";

    // create a new trajectory in SE(3)
    const auto nextContactPtr = m_contactList.getNextContact(m_state.time);
    assert(nextContactPtr != m_contactList.cend());

    log()->error("{} Creating a new SE(3) trajectory. Initial time {}, final time {}. Pose {}",
                 logPrefix,
                 m_state.time,
                 nextContactPtr->activationTime,
                 nextContactPtr->pose.coeffs().transpose());

    if (nextContactPtr == m_contactList.cend())
    {
        log()->error("{} Invalid next contact. Time {}.", logPrefix, m_state.time);
        return false;
    }

    // The rotation cannot change when the contact list is updated. For this reason we can build the
    // trajectory considering the initial and the final conditions
    m_staringTimeOfCurrentSO3Traj = m_state.time;
    const auto T = nextContactPtr->activationTime - m_staringTimeOfCurrentSO3Traj;
    m_SO3Planner.setInitialConditions(initialAngularVelocity, initialAngularAcceleration);
    m_SO3Planner.setFinalConditions(manif::SO3d::Tangent::Zero(), manif::SO3d::Tangent::Zero());
    if (!m_SO3Planner.setRotations(initialPose.quat(), nextContactPtr->pose.asSO3(), T))
    {
        log()->error("{} Unable to set the initial and final rotations for the SO(3) planner.",
                     logPrefix);
        return false;
    }

    if (!m_planarPlanner->setInitialConditions(initialPlanarVelocity, initialPlanarAcceleration))
    {
        log()->error("{} Unable to set the initial conditions for the planar planner.", logPrefix);
        return false;
    }
    if (!m_planarPlanner->setFinalConditions(Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()))
    {
        log()->error("{} Unable to set the final conditions for the planar planner.", logPrefix);
        return false;
    }

    // for the planar case we start at the current state at the given time
    if (!m_planarPlanner->setKnots({initialPose.translation().head<2>(),
                                    nextContactPtr->pose.translation().head<2>()},
                                   {m_state.time, nextContactPtr->activationTime}))
    {
        log()->error("{} Unable to set the knots for the planar planner.", logPrefix);
        return false;
    }

    // The foot maximum height point is given by the highest point plus the offset
    // If the robot is walking on a plane with height equal to zero, the footHeightViaPointPos is
    // given by the stepHeight
    const double footHeightViaPointPos
        = std::max(m_lastValidContact.pose.translation()(2), nextContactPtr->pose.translation()(2))
          + m_stepHeight;

    // the cast is required since m_footApexTime is a floating point number between 0 and 1
    const std::chrono::nanoseconds swingFootDuration = nextContactPtr->activationTime - m_lastValidContact.deactivationTime;
    const std::chrono::nanoseconds footHeightViaPointTime
        = std::chrono::duration_cast<std::chrono::nanoseconds>(
            m_footApexTime * swingFootDuration + m_lastValidContact.deactivationTime);

    if (!m_heightPlanner->setInitialConditions(initialVerticalVelocity,
                                               initialVerticalAcceleration))
    {
        log()->error("{} Unable to set the initial conditions for the height planner.", logPrefix);
        return false;
    }
    if (!m_heightPlanner->setFinalConditions(Vector1d::Constant(m_footLandingVelocity),
                                             Vector1d::Constant(m_footLandingAcceleration)))
    {
        log()->error("{} Unable to set the final conditions for the height planner.", logPrefix);
        return false;
    }

    if (m_state.time < footHeightViaPointTime)
    {
        if (!m_heightPlanner->setKnots({initialPose.translation().tail<1>(),
                                        Vector1d::Constant(footHeightViaPointPos),
                                        nextContactPtr->pose.translation().tail<1>()},
                                       {m_state.time,
                                        footHeightViaPointTime,
                                        nextContactPtr->activationTime}))
        {
            log()->error("{} Unable to set the knots for the knots for the height planner.",
                         logPrefix);
            return false;
        }
    } else
    {
        if (!m_heightPlanner->setKnots({initialPose.translation().tail<1>(),
                                        nextContactPtr->pose.translation().tail<1>()},
                                       {m_state.time, nextContactPtr->activationTime}))
        {
            log()->error("{} Unable to set the knots for the knots for the height planner.",
                         logPrefix);
            return false;
        }
    }

    return true;
}

bool SwingFootPlanner::evaluateSE3Traj(BipedalLocomotion::Planners::SwingFootPlannerState& state)
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
    auto angularVelocity(state.mixedVelocity.asSO3());
    auto angularAcceleration(state.mixedAcceleration.asSO3());
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
                                        state.mixedVelocity.coeffs().head<2>(),
                                        state.mixedAcceleration.coeffs().head<2>()))
    {
        log()->error("{} Unable to get the planar trajectory at time t = {}.",
                     logPrefix,
                     m_currentTrajectoryTime);
        return false;
    }

    if (!m_heightPlanner->evaluatePoint(m_currentTrajectoryTime,
                                        position.segment<1>(2),
                                        state.mixedVelocity.coeffs().segment<1>(2),
                                        state.mixedAcceleration.coeffs().segment<1>(2)))
    {
        log()->error("{} Unable to the height trajectory at time t = {}.",
                     logPrefix,
                     m_currentTrajectoryTime);
        return false;
    }

    // set the transform
    state.transform = manif::SE3d(position, rotation);

    return true;
}

bool SwingFootPlanner::advance()
{
    constexpr auto logPrefix = "[SwingFootPlanner::advance]";

    m_isOutputValid = false;

    // update the time
    m_state.time = m_currentTrajectoryTime;

    if (m_contactList.size() == 0)
    {
        log()->error("{} Empty contact list. Please call SwingFootPlanner::setContactList()",
                     logPrefix);
        return false;
    }

    // here there are several case
    // 0. the current time exceed the last deactivation time. This means that the trajectory ended
    // in this case lets assume that the output is valid and we avoid to update the state
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
        m_isOutputValid = true;

        // update the internal time
        m_currentTrajectoryTime += m_dT;
        return true;
    }

    // This is the case (1). In this case we do not have to update the state
    if (m_state.isInContact && m_currentTrajectoryTime < m_lastValidContact.deactivationTime)
    {
        m_isOutputValid = true;

        // update the internal time
        m_currentTrajectoryTime += m_dT;
        return true;
    }

    // This is the case (2)
    if (m_state.isInContact && m_currentTrajectoryTime >= m_lastValidContact.deactivationTime)
    {
        if (!this->createSE3Traj(m_state.transform,
                                 Eigen::Vector2d::Zero(),
                                 Eigen::Vector2d::Zero(),
                                 Vector1d::Constant(m_footTakeOffVelocity),
                                 Vector1d::Constant(m_footTakeOffAcceleration),
                                 manif::SO3d::Tangent::Zero(),
                                 manif::SO3d::Tangent::Zero()))
        {
            log()->error("{} Unable to create the new SE(3) trajectory.", logPrefix);
            return false;
        }

        if (!this->evaluateSE3Traj(m_state))
        {
            log()->error("{} Unable to update the SE(3) trajectory.", logPrefix);
            return false;
        }

        // the input was in contact but now it is not
        m_state.isInContact = false;
        m_isOutputValid = true;

        // update the internal time
        m_currentTrajectoryTime += m_dT;
        return true;
    }

    // This is the case (3)
    auto nextContact = m_contactList.getNextContact(m_currentTrajectoryTime);
    if (nextContact == m_contactList.end())
    {
        log()->error("{} Unable to get the next contact.", logPrefix);
        return false;
    }

    if (!m_state.isInContact && m_currentTrajectoryTime > m_lastValidContact.deactivationTime
        && m_currentTrajectoryTime < nextContact->activationTime)
    {
        if (!this->evaluateSE3Traj(m_state))
        {
            log()->error("{} Unable to update the SE(3) trajectory.", logPrefix);
            return false;
        }

        // now the contact is broken
        m_state.isInContact = false;

        m_isOutputValid = true;

        // update the internal time
        m_currentTrajectoryTime += m_dT;
        return true;
    }

    // This is the case (4)
    log()->error("m_state.isInContact {}, m_currentTrajectoryTime {} "
                 "m_lastValidContact.deactivationTime {} nextContact->activationTime {}",
                 m_state.isInContact,
                 m_currentTrajectoryTime,
                 m_lastValidContact.deactivationTime,
                 nextContact->activationTime);


    // m_state.isInContact false, m_currentTrajectoryTime 9000000000ns m_lastValidContact.deactivationTime 13700000000ns nextContact->activationTime 9000000000ns
    if (!m_state.isInContact && m_currentTrajectoryTime > m_lastValidContact.deactivationTime
        && m_currentTrajectoryTime >= nextContact->activationTime)
    {
        m_lastValidContact = *nextContact;
        log()->warn("caso 3");

        m_state.transform = m_lastValidContact.pose;
        m_state.mixedVelocity.setZero();
        m_state.mixedAcceleration.setZero();
        m_state.isInContact = true;

        m_isOutputValid = true;

        // update the internal time
        m_currentTrajectoryTime += m_dT;
        return true;
    }

    log()->error("{} Unable to evaluate the next step in the swing foot planner. This should never "
                 "happen.",
                 logPrefix);
    return false;
}
