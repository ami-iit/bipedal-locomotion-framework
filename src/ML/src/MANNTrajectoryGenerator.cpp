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

#include <iDynTree/Model.h>
#include <manif/SE3.h>

#include <iDynTree/EigenHelpers.h>

using namespace BipedalLocomotion::ML;
using namespace BipedalLocomotion;

struct MANNTrajectoryGenerator::Impl
{
    template <typename T> struct ScalingState
    {
        T previous;
        T previousScaled;
    };

    struct MANNTrajectoryGeneratorState
    {
        MANNAutoregressive::AutoregressiveState MANNAutoregressiveState;
        ScalingState<Eigen::Vector3d> com;
        ScalingState<Eigen::Vector3d> basePosition;
    };

    MANNAutoregressive mannAutoregressive;
    MANNAutoregressiveInput mannAutoregressiveInput;
    std::vector<MANNTrajectoryGeneratorState> mergePointStates;
    std::unordered_map<std::string, std::vector<ScalingState<Eigen::Vector3d>>> footState;

    std::chrono::nanoseconds dT;
    bool isOutputValid{false};
    double slowDownFactor{1.0};
    double scalingFactor{1.0};

    Contacts::ContactListMap contactListMap;

    MANNTrajectoryGeneratorOutput output;
    std::chrono::nanoseconds horizon;

    iDynTree::KinDynComputations kinDyn;
    Eigen::Vector3d gravity;
    int leftFootIndex;
    int rightFootIndex;

    /**
     * Reset the contact list given an estimated contact from mann autorergressive.
     */
    bool resetContactList(const std::string& contactName,
                          const Contacts::EstimatedContact& estimatedContact,
                          const std::chrono::nanoseconds& time);

    /**
     * Update the contact list given an estimated contact from mann autorergressive.
     */
    bool updateContactList(const std::string& contactName,
                           const Contacts::EstimatedContact& estimatedContact,
                           const std::chrono::nanoseconds& time);

    static double extractYawAngle(const Eigen::Ref<const Eigen::Matrix3d>& R);
};

double MANNTrajectoryGenerator::Impl::extractYawAngle(const Eigen::Ref<const Eigen::Matrix3d>& R)
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
    const std::string& contactName,
    const Contacts::EstimatedContact& estimatedContact,
    const std::chrono::nanoseconds& time)
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

        const Contacts::PlannedContact temp = *latestActiveContact;
        this->contactListMap[contactName].clear();
        return this->contactListMap[contactName].addContact(temp);
    }

    // if the contact is active we add a new contact to the list
    Contacts::PlannedContact temp;
    temp.activationTime = estimatedContact.switchTime;
    temp.deactivationTime = std::chrono::nanoseconds::max();
    temp.index = estimatedContact.index;
    temp.name = estimatedContact.name;

    // Here we assume that the ground is flat. Therefore, we can set the z to zero
    const double yaw = this->extractYawAngle(estimatedContact.pose.quat().toRotationMatrix());
    temp.pose.translation(
        {estimatedContact.pose.translation()[0], estimatedContact.pose.translation()[1], 0.0});
    temp.pose.quat(Eigen::AngleAxis(yaw, Eigen::Vector3d::UnitZ()));
    temp.type = Contacts::ContactType::FULL;

    this->contactListMap[contactName].clear();
    return this->contactListMap[contactName].addContact(temp);
}

bool MANNTrajectoryGenerator::Impl::updateContactList(
    const std::string& contactName,
    const Contacts::EstimatedContact& estimatedContact,
    const std::chrono::nanoseconds& time)
{
    constexpr auto logPrefix = "[MANNTrajectoryGenerator::Impl::updateContactList]";

    auto contactListIt = this->contactListMap.find(contactName);
    if (contactListIt == this->contactListMap.end())
    {
        log()->error("{} Unable to find the contact named {} in the dictionary.",
                     logPrefix,
                     contactName);
        return false;
    }
    Contacts::ContactList& contactList = contactListIt->second;

    // if the contact list map is empty then this is the first contact that we have to add to the
    // list read it as (or)
    // 1. if contact is active and the list is empty means that the foot just touched the ground
    // 2. if contact is active and the last contact is not active means that the foot just touched
    // the ground
    if (estimatedContact.isActive
        && (contactList.size() == 0 || !contactList.lastContact()->isContactActive(time)))
    {
        Contacts::PlannedContact temp;
        temp.activationTime = estimatedContact.switchTime;
        temp.deactivationTime = std::chrono::nanoseconds::max();
        temp.index = estimatedContact.index;
        temp.name = estimatedContact.name;

        // Here we assume that the ground is flat. Therefore, we can set the z to zero
        const double yaw = Impl::extractYawAngle(estimatedContact.pose.quat().toRotationMatrix());
        temp.pose.translation(
            {estimatedContact.pose.translation()[0], estimatedContact.pose.translation()[1], 0.0});
        temp.pose.quat(Eigen::AngleAxis(yaw, Eigen::Vector3d::UnitZ()));
        temp.type = Contacts::ContactType::FULL;
        if (!contactList.addContact(temp))
        {
            log()->error("{} Unable to update the contact list for the contact named '{}",
                         logPrefix,
                         contactName);
            return false;
        }
    }
    // In this case the contact ended so we have to set the deactivation time
    else if (!estimatedContact.isActive && contactList.size() != 0
             && contactList.lastContact()->isContactActive(time))
    {
        // we cannot update the status of the last contact. We need to remove it and add it again
        Contacts::PlannedContact lastContact = *(contactListMap[contactName].lastContact());
        lastContact.deactivationTime = time;
        contactList.removeLastContact();
        if (!contactList.addContact(lastContact))
        {
            log()->error("{} Unable to update the contact list for the contact named '{}",
                         logPrefix,
                         contactName);
            return false;
        }
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
    return m_pimpl->mannAutoregressive.setRobotModel(model)
           && m_pimpl->kinDyn.loadRobotModel(model);
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

    // the following lambda function is useful to get the name of the indexes of all the frames
    // required by the class
    auto getFrameIndex
        = [this,
           getParameter,
           logPrefix](std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                      const std::string& frameParamName,
                      int& frameIndex) -> bool {
        std::string frameName;
        if (!getParameter(handler, frameParamName, frameName))
        {
            log()->error("{} Unable to find the parameter named '{}'.", logPrefix, frameParamName);
            return false;
        }

        iDynTree::FrameIndex tmp = this->m_pimpl->kinDyn.model().getFrameIndex(frameName);
        if (tmp == iDynTree::LINK_INVALID_INDEX)
        {
            log()->error("{} Unable to find the frame named '{}' in the model.",
                         logPrefix,
                         frameName);
            return false;
        }

        frameIndex = tmp;

        return true;
    };

    bool ok = getParameter(paramHandler, "slow_down_factor", m_pimpl->slowDownFactor);
    ok = ok && getParameter(paramHandler, "scaling_factor", m_pimpl->scalingFactor);
    ok = ok && getParameter(paramHandler, "time_horizon", m_pimpl->horizon);
    ok = ok && getParameter(paramHandler, "sampling_time", m_pimpl->dT);
    ok = ok && getFrameIndex(paramHandler, "left_foot_frame_name", m_pimpl->leftFootIndex);
    ok = ok && getFrameIndex(paramHandler, "right_foot_frame_name", m_pimpl->rightFootIndex);

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
    m_pimpl->output.timestamps.resize(m_pimpl->horizon / m_pimpl->dT);

    m_pimpl->gravity.setZero();
    m_pimpl->gravity(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;
    return m_pimpl->mannAutoregressive.initialize(paramHandler);
}

bool MANNTrajectoryGenerator::setInitialState(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                                              const manif::SE3d& basePose)
{
    constexpr auto logPrefix = "[MANNTrajectoryGenerator::setInitialState]";

    Impl::MANNTrajectoryGeneratorState trajectoryGeneratorState;
    if (!m_pimpl->mannAutoregressive
             .populateInitialAutoregressiveState(jointPositions,
                                                 basePose,
                                                 trajectoryGeneratorState.MANNAutoregressiveState))
    {
        log()->error("{} Unable to populate the initial autoregressive state.", logPrefix);
        return false;
    }

    // compute the CoM position
    Eigen::Matrix<double, 6, 1> baseVelocity;
    baseVelocity.setZero();
    const Eigen::VectorXd jointVelocities = Eigen::VectorXd::Zero(jointPositions.size());
    if (!m_pimpl->kinDyn.setRobotState(basePose.transform(),
                                       jointPositions,
                                       baseVelocity,
                                       jointVelocities,
                                       m_pimpl->gravity))
    {
        log()->error("{} Unable to reset the kindyncomputations object.", logPrefix);
        return false;
    }

    // at the beginning the scaled and the unscaled position are the same
    trajectoryGeneratorState.com.previous
        = iDynTree::toEigen(m_pimpl->kinDyn.getCenterOfMassPosition());
    trajectoryGeneratorState.com.previousScaled = trajectoryGeneratorState.com.previous;

    trajectoryGeneratorState.basePosition.previous = basePose.translation();
    trajectoryGeneratorState.basePosition.previousScaled
        = trajectoryGeneratorState.basePosition.previous;

    // reset the contact list
    m_pimpl->footState.clear();

    // add the first contact
    Impl::ScalingState<Eigen::Vector3d> leftScalingState;
    leftScalingState.previous
        = trajectoryGeneratorState.MANNAutoregressiveState.leftFootState.contact.pose.translation();
    leftScalingState.previousScaled = leftScalingState.previous;
    m_pimpl->footState[trajectoryGeneratorState.MANNAutoregressiveState.leftFootState.contact.name]
        .push_back(std::move(leftScalingState));

    Impl::ScalingState<Eigen::Vector3d> rightScalingState;
    rightScalingState.previous
        = trajectoryGeneratorState.MANNAutoregressiveState.rightFootState.contact.pose.translation();
    rightScalingState.previousScaled = rightScalingState.previous;
    m_pimpl->footState[trajectoryGeneratorState.MANNAutoregressiveState.rightFootState.contact.name]
        .push_back(std::move(rightScalingState));

    for (auto& state : m_pimpl->mergePointStates)
    {
        state = trajectoryGeneratorState;
    }

    return true;
}

bool MANNTrajectoryGenerator::setInput(const Input& input)
{
    constexpr auto logPrefix = "[MANNTrajectoryGenerator::setInput]";

    if (input.mergePointIndex >= m_pimpl->mergePointStates.size())
    {
        log()->error("{} The index of the merge point is greater than the entire trajectory.  The "
                     "trajectory lasts {}. I cannot attach a new trajectory at {}.",
                     logPrefix,
                     m_pimpl->horizon,
                     input.mergePointIndex * m_pimpl->dT);
        return false;
    }

    m_pimpl->mannAutoregressiveInput = input;
    const Impl::MANNTrajectoryGeneratorState& mergePointState
        = m_pimpl->mergePointStates[input.mergePointIndex];

    m_pimpl->mergePointStates[0].com = mergePointState.com;
    m_pimpl->mergePointStates[0].basePosition = mergePointState.basePosition;

    // reset the MANN
    if (!m_pimpl->mannAutoregressive.reset(mergePointState.MANNAutoregressiveState))
    {
        log()->error("{} Unable to reset MANN.", logPrefix);
        return false;
    }

    const auto& time = mergePointState.MANNAutoregressiveState.time;
    for (const auto& [contactName, contactList] : m_pimpl->contactListMap)
    {
        std::size_t contactIndex = 0;
        auto contactIt = contactList.getActiveContact(time);
        if (contactIt != contactList.cend())
        {
            contactIndex = std::distance(contactList.cbegin(), contactIt);
        } else
        {
            contactIt = contactList.getNextContact(time);
            if (contactIt != contactList.cend())
            {
                contactIndex = std::distance(contactList.cbegin(), contactIt);
            } else
            {
                contactIndex = contactList.size();
            }
        }

        Impl::ScalingState<Eigen::Vector3d> scalingState
            = m_pimpl->footState[contactName][contactIndex];
        m_pimpl->footState[contactName].clear();
        m_pimpl->footState[contactName].push_back(std::move(scalingState));
    }

    // add the first contact if needed
    return m_pimpl->resetContactList("left_foot",
                                     mergePointState.MANNAutoregressiveState.leftFootState.contact,
                                     mergePointState.MANNAutoregressiveState.time)
           && m_pimpl
                  ->resetContactList("right_foot",
                                     mergePointState.MANNAutoregressiveState.rightFootState.contact,
                                     mergePointState.MANNAutoregressiveState.time);
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
        // we need to get the autoregressive state from the MANNAutoregressive before setting the
        // input and advancing the system. Indeed the autoregressive state at time t is the one
        // that will be used to generate the output at time t
        m_pimpl->mergePointStates[i].MANNAutoregressiveState
            = m_pimpl->mannAutoregressive.getAutoregressiveState();

        if (!m_pimpl->mannAutoregressive.setInput(m_pimpl->mannAutoregressiveInput))
        {
            log()->error("{} Unable to set the input of MANN at iteration number {}.",
                         logPrefix,
                         i);
            return false;
        }

        if (!m_pimpl->mannAutoregressive.advance())
        {
            log()->error("{} Unable to perform the iteration number {} of MANN.", logPrefix, i);
            return false;
        }

        if (!m_pimpl->mannAutoregressive.isOutputValid())
        {
            log()->error("{} The output of MANN is not valid at iteration number {}.",
                         logPrefix,
                         i);
            return false;
        }
        const auto& MANNOutput = m_pimpl->mannAutoregressive.getOutput();

        // populate the output of the trajectory generator
        m_pimpl->output.jointPositions[i] = MANNOutput.jointsPosition;
        m_pimpl->output.angularMomentumTrajectory[i] = MANNOutput.angularMomentum;
        m_pimpl->output.basePoses[i] = MANNOutput.basePose;
        m_pimpl->output.comTrajectory[i] = MANNOutput.comPosition;
        m_pimpl->output.timestamps[i] = MANNOutput.currentTime;

        // update the contacts lists
        if (!m_pimpl->updateContactList("left_foot", MANNOutput.leftFoot, MANNOutput.currentTime))
        {
            log()->error("{} Unable to update the contact list for the left_foot at iteration "
                         "number {}.",
                         logPrefix,
                         i);
            return false;
        }
        if (!m_pimpl->updateContactList("right_foot", MANNOutput.rightFoot, MANNOutput.currentTime))
        {
            log()->error("{} Unable to update the contact list for the right_foot at iteration "
                         "number {}.",
                         logPrefix,
                         i);
            return false;
        }

        // the next previous position is the current position
        if (i < m_pimpl->mergePointStates.size() - 1)
        {
            m_pimpl->mergePointStates[i + 1].com.previous = MANNOutput.comPosition;
            m_pimpl->mergePointStates[i + 1].basePosition.previous
                = MANNOutput.basePose.translation();
        }
    }

    // The following code is required to scale the output of the trajectory generator
    // In particular we need to scale the following quantities:
    // - angular momentum
    // - com trajectory
    // - base position
    // - contact position
    // - the time stamps

    // scale the com trajectory, the base position, the angular momentum nad the time stamps
    // To scale the CoM and the base position we use the following formula:
    //      x_scaled = x_previous_scaled + scaling_factor * (x - x_previous)
    for (int i = 0; i < m_pimpl->mergePointStates.size(); i++)
    {
        // scale the time stamps associated to the signal vectors.
        // The time stamps are scaled using the following formula
        // t_scaled = t * slow_down_factor
        m_pimpl->output.timestamps[i] *= m_pimpl->slowDownFactor;

        // scale the angular momentum trajectory considering the time slow down factor
        // h_scaled = h / slow_down_factor
        // if the slow down factor is equal to 1.0 then the angular momentum is not scaled
        // if the slow down factor is greater than 1.0 then the angular momentum is scaled
        m_pimpl->output.angularMomentumTrajectory[i] /= m_pimpl->slowDownFactor;

        // evaluate the com scaled
        m_pimpl->output.comTrajectory[i] = m_pimpl->mergePointStates[i].com.previousScaled
                                           + m_pimpl->scalingFactor
                                                 * (m_pimpl->output.comTrajectory[i]
                                                    - m_pimpl->mergePointStates[i].com.previous);

        // evaluate the base position scaled
        const Eigen::Vector3d scaledBasePosition
            = m_pimpl->mergePointStates[i].basePosition.previousScaled
              + m_pimpl->scalingFactor
                    * (m_pimpl->output.basePoses[i].translation()
                       - m_pimpl->mergePointStates[i].basePosition.previous);
        m_pimpl->output.basePoses[i].translation(scaledBasePosition);

        // update the previous position considering that the next previous is the current
        if (i < m_pimpl->mergePointStates.size() - 1)
        {
            m_pimpl->mergePointStates[i + 1].com.previousScaled = m_pimpl->output.comTrajectory[i];

            m_pimpl->mergePointStates[i + 1].basePosition.previousScaled
                = m_pimpl->output.basePoses[i].translation();
        }
    }

    // scaled contact list map
    Contacts::ContactListMap scaledContactListMap;
    for (const auto& [contactName, contactList] : m_pimpl->contactListMap)
    {
        for (int i = 0; i < contactList.size(); i++)
        {
            // create the scaled contact (we need to scale the activation and deactivation time)
            Contacts::PlannedContact scaledContact = contactList[i];
            scaledContact.activationTime *= m_pimpl->slowDownFactor;
            if (scaledContact.deactivationTime != std::chrono::nanoseconds::max())
            {
                scaledContact.deactivationTime *= m_pimpl->slowDownFactor;
            }

            // scale the position
            const Eigen::Vector3d scaledPosition
                = m_pimpl->footState[contactName][i].previousScaled
                  + m_pimpl->scalingFactor
                        * (contactList[i].pose.translation()
                           - m_pimpl->footState[contactName][i].previous);
            scaledContact.pose.translation(scaledPosition);

            // add the contact to the scaled contact list
            if (!scaledContactListMap[contactName].addContact(scaledContact))
            {
                log()->error("{} Unable to add the contact named {} to the scaled contact list.",
                             logPrefix,
                             contactName);
                return false;
            }

            // update the previous position considering that the next previous is the current
            Impl::ScalingState<Eigen::Vector3d> scaledContactState;
            scaledContactState.previousScaled = scaledPosition;
            scaledContactState.previous = contactList[i].pose.translation();
            m_pimpl->footState[contactName].push_back(std::move(scaledContactState));
        }
    }

    // populate the phase list with the scaled contact list map
    m_pimpl->output.phaseList.setLists(scaledContactListMap);

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
