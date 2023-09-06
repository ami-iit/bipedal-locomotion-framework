/**
 * @file MANNTrajectoryGenerator.cpp
 * @authors Paolo Maria Viceconte, Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Contacts/ContactList.h>
#include <chrono>

#include <BipedalLocomotion/ML/MANNAutoregressive.h>
#include <BipedalLocomotion/ML/MANNTrajectoryGenerator.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/Model/Model.h>
#include <manif/SE3.h>

using namespace BipedalLocomotion::ML;
using namespace BipedalLocomotion;

struct MANNTrajectoryGenerator::Impl
{
    struct MergePointState
    {
        MANNInput input;
        Contacts::EstimatedContact leftFoot;
        Math::SchmittTriggerState leftFootSchmittTriggerState;
        Contacts::EstimatedContact rightFoot;
        Math::SchmittTriggerState rightFootSchmittTriggerState;
        manif::SE3d basePosition;
        manif::SE3Tangentd baseVelocity;
        MANNAutoregressive::AutoregressiveState autoregressiveState;

        /** Planar position of the previous left foot contact*/
        Eigen::Vector2d I_p_left_previous;

        /** Planar position of the previous right foot contact*/
        Eigen::Vector2d I_p_right_previous;

        Eigen::Vector2d I_p_left_scaled_previous; /**< Planar position of the previous left foot
                                                     contact scaled.*/
        Eigen::Vector2d I_p_right_scaled_previous; /**< Planar position of the previous right foot
                                                      contact scaled.*/

        /** TODO */
        Eigen::Vector2d I_p_com_previous;

        /** TODO */
        Eigen::Vector2d I_p_base_previous;

        Eigen::Vector2d I_p_com_scaled_previous; /**< Planar position of the previous com scaled.*/
        Eigen::Vector2d I_p_base_scaled_previous; /**< Planar position of the previous base
                                                     scaled.*/

        std::chrono::nanoseconds time;
    };

    MANNAutoregressive mann;
    MANNAutoregressiveInput mannAutoregressiveInput;
    std::vector<MergePointState> mergePointStates;
    std::chrono::nanoseconds dT;
    bool isOutputValid{false};
    int slowDownFactor{1};
    double scalingFactor{1};

    Contacts::ContactListMap contactListMap;
    MANNTrajectoryGeneratorOutput output;
    std::chrono::nanoseconds horizon;
    int projectedBaseHorizonSize;

    bool updateContactList(const std::string& footName,
                           const std::chrono::nanoseconds& currentTime,
                           const Contacts::EstimatedContact& estimatedContact,
                           Eigen::Ref<const Eigen::Vector2d> I_p_contact_previous,
                           Eigen::Ref<const Eigen::Vector2d> I_p_contact_scaled_previous,
                           Eigen::Ref<Eigen::Vector2d> I_p_contact_scaled_previous_updated);

    bool resetContactList(const Contacts::EstimatedContact& estimatedContact,
                          const std::chrono::nanoseconds& time,
                          const std::string& contactName,
                          Eigen::Ref<const Eigen::Vector2d> I_p_foot_previous,
                          Eigen::Ref<const Eigen::Vector2d> I_p_contact_scaled_previous);

    static double extactYawAngle(const Eigen::Ref<const Eigen::Matrix3d>& R);
};

double MANNTrajectoryGenerator::Impl::extactYawAngle(const Eigen::Ref<const Eigen::Matrix3d>& R)
{
    if (R(2, 0) < 1.0)
    {
        if (R(2, 0) > -1.0)
        {
            return atan2(R(1, 0), R(0, 0));
        } else
        {
            // Not a unique solution
            return -atan2(-R(1, 2), R(1, 1));
        }
    }

    // Not a unique solution
    return atan2(-R(1, 2), R(1, 1));
}

bool MANNTrajectoryGenerator::Impl::resetContactList(
    const Contacts::EstimatedContact& estimatedContact,
    const std::chrono::nanoseconds& time,
    const std::string& contactName,
    Eigen::Ref<const Eigen::Vector2d> I_p_foot_previous,
    Eigen::Ref<const Eigen::Vector2d> I_p_contact_scaled_previous)
{
    auto& contactList = this->contactListMap[contactName];

    constexpr auto logPrefix = "[MANNTrajectoryGenerator::Impl::resetContactList]";

    // if the contact is not active we store the previous valid contact in the contact list
    if (!estimatedContact.isActive)
    {
        // if the contact is not active we keep the previous contact in the contact list
        const auto latestActiveContact = contactList.getPresentContact(time);

        if (latestActiveContact == contactList.cend())
        {
            log()->error("[MANNTrajectoryGenerator::Impl::resetContactList] Unable to find the "
                         "latest active contact for the contact named {}. Time {}.",
                         contactName,
                         std::chrono::duration_cast<std::chrono::milliseconds>(time));

            for (const auto& contact : contactList)
            {
                log()->error("[MANNTrajectoryGenerator::Impl::resetContactList] Contact {} is "
                             "active between {} and {}.",
                             contact.name,
                             std::chrono::duration_cast<std::chrono::milliseconds>(
                                 contact.activationTime),
                             std::chrono::duration_cast<std::chrono::milliseconds>(
                                 contact.deactivationTime));
            }
            return false;
        }

        Contacts::PlannedContact temp = *latestActiveContact;
        this->contactListMap[contactName].clear();
        return this->contactListMap[contactName].addContact(temp);
    }

    // if the contact is active we add a new contact to the list
    Contacts::PlannedContact temp;
    temp.activationTime = estimatedContact.switchTime * slowDownFactor;
    temp.deactivationTime = std::chrono::nanoseconds::max();
    temp.index = estimatedContact.index;
    temp.name = estimatedContact.name;

    const double yaw = this->extactYawAngle(estimatedContact.pose.quat().toRotationMatrix());
    Eigen::Vector2d originalPlanarPosition = estimatedContact.pose.translation().head<2>();

    const Eigen::Vector2d I_p_contact_scaled
        = I_p_contact_scaled_previous
          + this->scalingFactor * (originalPlanarPosition - I_p_foot_previous);

    temp.pose.translation({I_p_contact_scaled[0], I_p_contact_scaled[1], 0.0});
    temp.pose.quat(Eigen::AngleAxis(yaw, Eigen::Vector3d::UnitZ()));

    temp.type = Contacts::ContactType::FULL;

    this->contactListMap[contactName].clear();
    return this->contactListMap[contactName].addContact(temp);
}

bool MANNTrajectoryGenerator::Impl::updateContactList(
    const std::string& footName,
    const std::chrono::nanoseconds& currentTime,
    const Contacts::EstimatedContact& estimatedContact,
    Eigen::Ref<const Eigen::Vector2d> I_p_contact_previous,
    Eigen::Ref<const Eigen::Vector2d> I_p_contact_scaled_previous,
    Eigen::Ref<Eigen::Vector2d> I_p_contact_scaled_previous_updated)
{
    constexpr auto logPrefix = "[MANNTrajectoryGenerator::Impl::updateContactList]";

    auto contactListIt = this->contactListMap.find(footName);
    if (contactListIt == this->contactListMap.end())
    {
        log()->error("{} Unable to find the contact named {} in the dictionary.",
                     logPrefix,
                     footName);
        return false;
    }
    Contacts::ContactList& contactList = contactListIt->second;

    I_p_contact_scaled_previous_updated = I_p_contact_scaled_previous;

    // if the contact list map is empty then this is the first contact that we have to add to the
    // list read it as (or)
    // 1. if contact is active and the list is empty means that the foot just touched the ground
    // 2. if contact is active and the last contact is not active means that the foot just touched
    // the ground
    if (estimatedContact.isActive
        && (contactList.size() == 0
            || !contactList.lastContact()->isContactActive(currentTime * this->slowDownFactor)))
    {
        Contacts::PlannedContact temp;
        temp.activationTime = estimatedContact.switchTime * this->slowDownFactor;
        temp.deactivationTime = std::chrono::nanoseconds::max();
        temp.index = estimatedContact.index;
        temp.name = estimatedContact.name;

        // force the foot to be attached to a flat terrain
        const double yaw = Impl::extactYawAngle(estimatedContact.pose.quat().toRotationMatrix());
        Eigen::Vector2d originalPlanarPosition = estimatedContact.pose.translation().head<2>();

        const Eigen::Vector2d I_p_contact_scaled
            = I_p_contact_scaled_previous
              + this->scalingFactor * (originalPlanarPosition - I_p_contact_previous);

        temp.pose.translation({I_p_contact_scaled[0], I_p_contact_scaled[1], 0.0});
        temp.pose.quat(Eigen::AngleAxis(yaw, Eigen::Vector3d::UnitZ()));

        temp.type = Contacts::ContactType::FULL;
        if (!contactList.addContact(temp))
        {
            log()->error("{} Unable to update the contact list for the contact named '{}",
                         logPrefix,
                         footName);
            return false;
        }
    }
    // In this case the contact ended so we have to set the deactivation time
    else if (!estimatedContact.isActive && contactList.size() != 0
             && contactList.lastContact()->isContactActive(currentTime * this->slowDownFactor))
    {
        // we cannot update the status of the last contact. We need to remove it and add it again
        Contacts::PlannedContact lastContact = *(contactListMap[footName].lastContact());
        lastContact.deactivationTime = currentTime * this->slowDownFactor;
        contactList.removeLastContact();
        if (!contactList.addContact(lastContact))
        {
            log()->error("{} Unable to update the contact list for the contact named '{}",
                         logPrefix,
                         footName);
            return false;
        }

        // thw pose did not changed and the contact has been deactivated so we can consider it as
        // previous contact
        I_p_contact_scaled_previous_updated = lastContact.pose.translation().head<2>();
    }

    return true;
}

MANNTrajectoryGenerator::MANNTrajectoryGenerator()
    : m_pimpl(std::make_unique<Impl>())
{
}

MANNTrajectoryGenerator::~MANNTrajectoryGenerator() = default;

bool MANNTrajectoryGenerator::setRobotModel(const iDynTree::Model& model)
{
    return m_pimpl->mann.setRobotModel(model);
}

bool MANNTrajectoryGenerator::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto logPrefix = "[MANNTrajectoryGenerator::initialize]";

    auto getParameter
        = [logPrefix](std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                      const std::string& paramName,
                      auto& param) -> bool {
        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            log()->error("{} Invalid parameters handler.", logPrefix);
            return false;
        }

        if (!ptr->getParameter(paramName, param))
        {
            log()->error("{} Unable to find the parameter named {}.", logPrefix, paramName);
            return false;
        }
        return true;
    };

    // this parameter is optional

    bool ok = getParameter(paramHandler, "slow_down_factor", m_pimpl->slowDownFactor);
    ok = ok && getParameter(paramHandler, "time_horizon", m_pimpl->horizon);
    ok = ok && getParameter(paramHandler, "sampling_time", m_pimpl->dT);
    ok = ok && getParameter(paramHandler, "scaling_factor", m_pimpl->scalingFactor);
    ok = ok
         && getParameter(paramHandler.lock()->getGroup("MANN"),
                         "projected_base_horizon",
                         m_pimpl->projectedBaseHorizonSize);

    if (!ok)
    {
        return false;
    }

    // resize the quantities
    m_pimpl->mergePointStates.resize(m_pimpl->horizon / m_pimpl->dT);
    m_pimpl->output.basePoses.resize(m_pimpl->horizon / m_pimpl->dT);
    m_pimpl->output.jointPositions.resize(m_pimpl->horizon / m_pimpl->dT);
    m_pimpl->output.angularMomentumTrajectory.resize(m_pimpl->horizon / m_pimpl->dT);
    m_pimpl->output.comTrajectory.resize(m_pimpl->horizon / m_pimpl->dT);

    return m_pimpl->mann.initialize(paramHandler);
}

void MANNTrajectoryGenerator::setInitialState(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                                              const Contacts::EstimatedContact& leftFoot,
                                              const Contacts::EstimatedContact& rightFoot,
                                              const manif::SE3d& basePose,
                                              Eigen::Ref<const Eigen::Vector3d> comPosition,
                                              const std::chrono::nanoseconds& time)
{
    Impl::MergePointState temp;
    temp.input.jointPositions = jointPositions;
    temp.input.jointVelocities = Eigen::VectorXd::Zero(jointPositions.size());
    temp.input.basePositionTrajectory = Eigen::MatrixXd::Zero(2, m_pimpl->projectedBaseHorizonSize);
    temp.input.baseVelocitiesTrajectory
        = Eigen::MatrixXd::Zero(2, m_pimpl->projectedBaseHorizonSize);
    temp.input.facingDirectionTrajectory.resize(2, m_pimpl->projectedBaseHorizonSize);
    for (int i = 0; i < m_pimpl->projectedBaseHorizonSize; i++)
    {
        temp.input.facingDirectionTrajectory.col(i) << 1.0, 0.0;
    }
    temp.basePosition = basePose;
    temp.baseVelocity = manif::SE3Tangentd::Zero();
    temp.leftFoot = leftFoot;
    temp.rightFoot = rightFoot;
    temp.time = time;

    // 51 is the length of 1 second of past trajectory stored in the autoregressive state. Since the
    // original mocap data are collected at 50 Hz, and therefore the trajectory generation is
    // assumed to proceed at 50 Hz, we need 50 datapoints to store the past second of trajectory.
    // Along with the present datapoint, they sum up to 51!
    constexpr size_t lengthOfPresentPlusPastTrajectory = 51;
    temp.autoregressiveState.pastProjectedBasePositions
        = std::deque<Eigen::Vector2d>{lengthOfPresentPlusPastTrajectory, Eigen::Vector2d{0.0, 0.0}};
    temp.autoregressiveState.pastProjectedBaseVelocity
        = std::deque<Eigen::Vector2d>{lengthOfPresentPlusPastTrajectory, Eigen::Vector2d{0.0, 0.0}};
    temp.autoregressiveState.pastFacingDirection
        = std::deque<Eigen::Vector2d>{lengthOfPresentPlusPastTrajectory, Eigen::Vector2d{1.0, 0.0}};
    temp.autoregressiveState.I_H_FD = manif::SE2d::Identity();
    temp.autoregressiveState.previousMannInput = temp.input;
    temp.leftFootSchmittTriggerState.state = leftFoot.isActive;
    temp.rightFootSchmittTriggerState.state = rightFoot.isActive;

    temp.I_p_left_scaled_previous = leftFoot.pose.translation().head<2>();
    temp.I_p_right_scaled_previous = rightFoot.pose.translation().head<2>();

    temp.I_p_left_previous = leftFoot.pose.translation().head<2>();
    temp.I_p_right_previous = rightFoot.pose.translation().head<2>();

    temp.I_p_com_scaled_previous = comPosition.head<2>();
    temp.I_p_base_scaled_previous = basePose.translation().head<2>();

    temp.I_p_base_previous = basePose.translation().head<2>();
    temp.I_p_com_previous = comPosition.head<2>();

    for (auto& state : m_pimpl->mergePointStates)
    {
        state = temp;
    }
}

bool MANNTrajectoryGenerator::setInput(const Input& input)
{
    constexpr auto logPrefix = "[MANNTrajectoryGenerator::setInput]";

    if (input.mergePointIndex > m_pimpl->mergePointStates.size())
    {
        log()->error("{} The index of the merge point is greater than the entire trajectory.  The "
                     "trajectory lasts {}. I cannot attach a new trajectory at {}.",
                     logPrefix,
                     m_pimpl->horizon,
                     input.mergePointIndex * m_pimpl->dT);
        return false;
    }

    m_pimpl->mannAutoregressiveInput = input;

    const auto& mergePointState = m_pimpl->mergePointStates[input.mergePointIndex];

    m_pimpl->mergePointStates[0].I_p_left_scaled_previous
        = mergePointState.I_p_left_scaled_previous;
    m_pimpl->mergePointStates[0].I_p_right_scaled_previous
        = mergePointState.I_p_right_scaled_previous;
    m_pimpl->mergePointStates[0].I_p_base_scaled_previous
        = mergePointState.I_p_base_scaled_previous;
    m_pimpl->mergePointStates[0].I_p_com_scaled_previous = mergePointState.I_p_com_scaled_previous;

    // reset the MANN
    if (!m_pimpl->mann.reset(mergePointState.input,
                             mergePointState.leftFoot,
                             mergePointState.leftFootSchmittTriggerState,
                             mergePointState.I_p_left_previous,
                             mergePointState.rightFoot,
                             mergePointState.rightFootSchmittTriggerState,
                             mergePointState.I_p_right_previous,
                             mergePointState.basePosition,
                             mergePointState.baseVelocity,
                             mergePointState.I_p_com_previous,
                             mergePointState.I_p_base_previous,
                             mergePointState.autoregressiveState,
                             mergePointState.time))
    {
        log()->error("{} Unable to reset MANN.", logPrefix);
        return false;
    }

    // add the first contact if needed
    return m_pimpl->resetContactList(mergePointState.leftFoot,
                                     mergePointState.time * m_pimpl->slowDownFactor,
                                     "left_foot",
                                     mergePointState.I_p_left_previous,
                                     mergePointState.I_p_left_scaled_previous)
           && m_pimpl->resetContactList(mergePointState.rightFoot,
                                        mergePointState.time * m_pimpl->slowDownFactor,
                                        "right_foot",
                                        mergePointState.I_p_right_previous,
                                        mergePointState.I_p_right_scaled_previous);
}

bool MANNTrajectoryGenerator::advance()
{
    constexpr auto logPrefix = "[MANNTrajectoryGenerator::advance]";

    // invalidate the output
    m_pimpl->isOutputValid = false;

    Eigen::Vector3d tempPosition;
    Eigen::Vector2d tempContactPositionScaled;

    for (int i = 0; i < m_pimpl->mergePointStates.size(); i++)
    {
        if (!m_pimpl->mann.setInput(m_pimpl->mannAutoregressiveInput))
        {
            log()->error("{} Unable to set the input to MANN.", logPrefix);
            return false;
        }

        if (!m_pimpl->mann.advance())
        {
            log()->error("{} Unable to perform the iteration number {} of MANN.", logPrefix, i);
            return false;
        }

        const auto& MANNOutput = m_pimpl->mann.getOutput();

        // store the status required to reset the system
        m_pimpl->mergePointStates[i].basePosition = MANNOutput.basePose;
        m_pimpl->mergePointStates[i].autoregressiveState = m_pimpl->mann.getAutoregressiveState();
        m_pimpl->mergePointStates[i].baseVelocity = MANNOutput.baseVelocity;
        m_pimpl->mergePointStates[i].input = m_pimpl->mann.getMANNInput();
        m_pimpl->mergePointStates[i].leftFoot = MANNOutput.leftFoot;
        m_pimpl->mergePointStates[i].leftFootSchmittTriggerState
            = MANNOutput.leftFootSchmittTriggerState;
        m_pimpl->mergePointStates[i].rightFoot = MANNOutput.rightFoot;
        m_pimpl->mergePointStates[i].rightFootSchmittTriggerState
            = MANNOutput.rightFootSchmittTriggerState;
        m_pimpl->mergePointStates[i].time = MANNOutput.currentTime;

        m_pimpl->mergePointStates[i].I_p_left_previous = MANNOutput.I_p_left_previous;
        m_pimpl->mergePointStates[i].I_p_right_previous = MANNOutput.I_p_right_previous;
        m_pimpl->mergePointStates[i].I_p_com_previous = MANNOutput.I_p_com_previous;
        m_pimpl->mergePointStates[i].I_p_base_previous = MANNOutput.I_p_base_previous;

        // populate the output of the trajectory generator
        m_pimpl->output.jointPositions[i] = MANNOutput.jointsPosition;
        m_pimpl->output.angularMomentumTrajectory[i] = MANNOutput.angularMomentum;

        // Scale the base
        m_pimpl->output.basePoses[i].quat(MANNOutput.basePose.quat());
        tempPosition(2) = MANNOutput.basePose.translation()(2);
        tempPosition.head<2>() = m_pimpl->scalingFactor
                                     * (MANNOutput.basePose.translation().head<2>()
                                        - m_pimpl->mergePointStates[i].I_p_base_previous)
                                 + m_pimpl->mergePointStates[i].I_p_base_scaled_previous;
        m_pimpl->output.basePoses[i].translation(tempPosition);

        if (i < m_pimpl->mergePointStates.size() - 1)
        {
            m_pimpl->mergePointStates[i + 1].I_p_base_scaled_previous = tempPosition.head<2>();
        }

        // scale the Com
        m_pimpl->output.comTrajectory[i](2) = MANNOutput.comPosition(2);
        m_pimpl->output.comTrajectory[i].head<2>()
            = m_pimpl->scalingFactor
                  * (MANNOutput.comPosition.head<2>()
                     - m_pimpl->mergePointStates[i].I_p_com_previous)
              + m_pimpl->mergePointStates[i].I_p_com_scaled_previous;

        if (i < m_pimpl->mergePointStates.size() - 1)
        {
            m_pimpl->mergePointStates[i].I_p_com_scaled_previous
                = m_pimpl->output.comTrajectory[i].head<2>();
        }

        if (!m_pimpl->updateContactList("left_foot",
                                        MANNOutput.currentTime,
                                        MANNOutput.leftFoot,
                                        MANNOutput.I_p_left_previous,
                                        m_pimpl->mergePointStates[i].I_p_left_scaled_previous,
                                        tempContactPositionScaled))
        {
            log()->error("{} Unable to update the contact list for the left_foot at iteration "
                         "number {}.",
                         logPrefix,
                         i);
            return false;
        }

        // this is needed to track the scaled contact position required to evaluate the scale
        // contact at i + 1.
        if (i < m_pimpl->mergePointStates.size() - 1)
        {
            m_pimpl->mergePointStates[i].I_p_left_scaled_previous = tempContactPositionScaled;
        }

        if (!m_pimpl->updateContactList("right_foot",
                                        MANNOutput.currentTime,
                                        MANNOutput.rightFoot,
                                        MANNOutput.I_p_right_previous,
                                        m_pimpl->mergePointStates[i].I_p_right_scaled_previous,
                                        tempContactPositionScaled))
        {
            log()->error("{} Unable to update the contact list for the right_foot at iteration "
                         "number {}.",
                         logPrefix,
                         i);
            return false;
        }

        // this is needed to track the scaled contact position required to evaluate the scale
        // contact at i + 1.
        if (i < m_pimpl->mergePointStates.size() - 1)
        {
            m_pimpl->mergePointStates[i].I_p_right_scaled_previous = tempContactPositionScaled;
        }
    }

    // populate the phase list
    m_pimpl->output.phaseList.setLists(m_pimpl->contactListMap);

    m_pimpl->isOutputValid = true;

    return true;
}

bool MANNTrajectoryGenerator::isOutputValid() const
{
    return m_pimpl->isOutputValid;
}

const MANNTrajectoryGenerator::Output& MANNTrajectoryGenerator::getOutput() const
{
    return m_pimpl->output;
}
