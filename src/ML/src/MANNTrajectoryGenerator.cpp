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
    MANNAutoregressive mann;
    MANNAutoregressiveInput mannAutoregressiveInput;
    std::vector<MANNAutoregressive::AutoregressiveState> mergePointStates;
    std::chrono::nanoseconds dT;
    bool isOutputValid{false};
    int slowDownFactor{1};

    Contacts::ContactListMap contactListMap;
    MANNTrajectoryGeneratorOutput output;
    std::chrono::nanoseconds horizon;
    int projectedBaseHorizonSize;

    iDynTree::KinDynComputations kinDyn;
    Eigen::Vector3d gravity;
    int leftFootIndex;
    int rightFootIndex;
    int mergePoint; // TODO remove me

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
    temp.activationTime = estimatedContact.switchTime * this->slowDownFactor;
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
        && (contactList.size() == 0
            || !contactList.lastContact()->isContactActive(time * this->slowDownFactor)))
    {
        Contacts::PlannedContact temp;
        temp.activationTime = estimatedContact.switchTime * this->slowDownFactor;
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
             && contactList.lastContact()->isContactActive(time * this->slowDownFactor))
    {
        // we cannot update the status of the last contact. We need to remove it and add it again
        Contacts::PlannedContact lastContact = *(contactListMap[contactName].lastContact());
        lastContact.deactivationTime = time * this->slowDownFactor;
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
    return m_pimpl->mann.setRobotModel(model) && m_pimpl->kinDyn.loadRobotModel(model);
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
    ok = ok && getParameter(paramHandler, "time_horizon", m_pimpl->horizon);
    ok = ok && getParameter(paramHandler, "sampling_time", m_pimpl->dT);
    ok = ok && getFrameIndex(paramHandler, "left_foot_frame_name", m_pimpl->leftFootIndex);
    ok = ok && getFrameIndex(paramHandler, "right_foot_frame_name", m_pimpl->rightFootIndex);
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
    m_pimpl->output.timeStamps.resize(m_pimpl->horizon / m_pimpl->dT);

    m_pimpl->gravity.setZero();
    m_pimpl->gravity(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;
    return m_pimpl->mann.initialize(paramHandler);
}

bool MANNTrajectoryGenerator::setInitialState(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                                              const manif::SE3d& basePose)
{
    constexpr auto logPrefix = "[MANNTrajectoryGenerator::setInitialState]";

    MANNAutoregressive::AutoregressiveState autoregressiveState;
    if (!m_pimpl->mann.populateInitialAutoregressiveState(jointPositions,
                                                          basePose,
                                                          autoregressiveState))
    {
        log()->error("[MANNAutoregressive::reset] Unable to populate the initial autoregressive "
                     "state.");
        return false;
    }

    for (auto& state : m_pimpl->mergePointStates)
    {
        state = autoregressiveState;
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
    const auto& mergePointState = m_pimpl->mergePointStates[input.mergePointIndex];
    m_pimpl->mergePoint = input.mergePointIndex;

    // reset the MANN
    if (!m_pimpl->mann.reset(mergePointState))
    {
        log()->error("{} Unable to reset MANN.", logPrefix);
        return false;
    }

    // add the first contact if needed
    return m_pimpl->resetContactList("left_foot",
                                     mergePointState.leftFootState.contact,
                                     mergePointState.time * m_pimpl->slowDownFactor)
           && m_pimpl->resetContactList("right_foot",
                                        mergePointState.rightFootState.contact,
                                        mergePointState.time * m_pimpl->slowDownFactor);
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
        // TODO can we move it after the advance?
        m_pimpl->mergePointStates[i] = m_pimpl->mann.getAutoregressiveState();

        if (!m_pimpl->mann.setInput(m_pimpl->mannAutoregressiveInput))
        {
            log()->error("{} Unable to set the input to MANN.", logPrefix);
            return false;
        }

        // TODO
        // if (i == 0 || i == m_pimpl->mergePoint)
        // {
        //     // print the input of mann
        //     const auto& mannInput = m_pimpl->mann.getMANNInput();
        //     log()->error("{} MANN input at iteration {}.", logPrefix, i);
        //     log()->info("{} base position trajectory: {}.",
        //                 logPrefix,
        //                 mannInput.basePositionTrajectory.transpose());
        //     log()->info("{} base velocity trajectory: {}.",
        //                 logPrefix,
        //                 mannInput.baseVelocitiesTrajectory.transpose());
        //     log()->info("{}  facing direction trajectory: {}.",
        //                 logPrefix,
        //                 mannInput.facingDirectionTrajectory.transpose());
        //     log()->info("{}  joints positions: {}.",
        //                 logPrefix,
        //                 mannInput.jointPositions.transpose());
        //     log()->info("{}  joints velocities: {}.",
        //                 logPrefix,
        //                 mannInput.jointVelocities.transpose());
        // }

        if (!m_pimpl->mann.advance())
        {
            log()->error("{} Unable to perform the iteration number {} of MANN.", logPrefix, i);
            return false;
        }

        const auto& MANNOutput = m_pimpl->mann.getOutput();

        // store the status required to reset the system

        // populate the output of the trajectory generator
        m_pimpl->output.jointPositions[i] = MANNOutput.jointsPosition;
        m_pimpl->output.angularMomentumTrajectory[i] = MANNOutput.angularMomentum;
        m_pimpl->output.basePoses[i] = MANNOutput.basePose;
        m_pimpl->output.comTrajectory[i] = MANNOutput.comPosition;
        m_pimpl->output.timeStamps[i] = MANNOutput.currentTime;

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
    }

    // // populate the time stamps
    // for (int i = 0; i < m_pimpl->mergePointStates.size(); i++)
    // {
    //     m_pimpl->output.timeStamps[i] *= m_pimpl->slowDownFactor;
    // }

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
