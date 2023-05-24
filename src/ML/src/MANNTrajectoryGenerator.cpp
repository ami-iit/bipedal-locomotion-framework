/**
 * @file MANNTrajectoryGenerator.cpp
 * @authors Paolo Maria Viceconte, Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

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
        Contacts::EstimatedContact rightFoot;
        manif::SE3d basePosition;
        manif::SE3Tangentd baseVelocity;
        MANNAutoregressive::AutoregressiveState autoregressiveState;
        std::chrono::nanoseconds time;
    };

    MANNAutoregressive mann;
    MANNAutoregressiveInput mannAutoregressiveInput;
    std::vector<MergePointState> mergePointStates;
    std::chrono::nanoseconds dT;
    bool isOutputValid{false};
    int slowDownFactor{1};

    Contacts::ContactListMap contactListMap;
    MANNTrajectoryGeneratorOutput output;
    std::chrono::nanoseconds horizon;
    int projectedBaseHorizonSize;

    bool updateContactList(const std::string& footName,
                           const std::chrono::nanoseconds& currentTime,
                           const Contacts::EstimatedContact& estimatedContact);

    void resetContactList(const Contacts::EstimatedContact& estimatedContact,
                          const std::string& contactName);
};

void MANNTrajectoryGenerator::Impl::resetContactList(
    const Contacts::EstimatedContact& estimatedContact, const std::string& contactName)
{
    // if the contact is not active we do not have to add in the contact list
    if (!estimatedContact.isActive)
    {
        return;
    }
    Contacts::PlannedContact temp;
    temp.activationTime = estimatedContact.switchTime * slowDownFactor;
    temp.deactivationTime = std::chrono::nanoseconds::max();
    temp.index = estimatedContact.index;
    temp.name = estimatedContact.name;

    const Eigen::Vector3d ypr
        = estimatedContact.pose.quat().toRotationMatrix().eulerAngles(2, 1, 0);
    const Eigen::Vector3d position = estimatedContact.pose.translation();
    temp.pose.translation({position[0], position[1], 0.0});
    temp.pose.quat(Eigen::AngleAxis(ypr(0), Eigen::Vector3d::UnitZ()));

    temp.type = Contacts::ContactType::FULL;
    this->contactListMap[contactName].addContact(temp);
}

bool MANNTrajectoryGenerator::Impl::updateContactList(
    const std::string& footName,
    const std::chrono::nanoseconds& currentTime,
    const Contacts::EstimatedContact& estimatedContact)
{
    // if the contact list map is empty then this is the first contact that we have to add to the
    // list
    // read it as
    // 1. if contact is active and the list is empty means that the foot just touched the ground
    // 2. if contact is active and the last contact is not active means that the foot just touched
    // the ground
    if (estimatedContact.isActive
        && (contactListMap[footName].size() == 0
            || !contactListMap[footName].lastContact()->isContactActive(currentTime
                                                                        * this->slowDownFactor)))
    {
        Contacts::PlannedContact temp;
        temp.activationTime = estimatedContact.switchTime * this->slowDownFactor;
        temp.deactivationTime = std::chrono::nanoseconds::max();
        temp.index = estimatedContact.index;
        temp.name = estimatedContact.name;
        temp.pose.translation(
            {estimatedContact.pose.translation()[0], estimatedContact.pose.translation()[1], 0.0});

        const Eigen::Vector3d ypr
            = estimatedContact.pose.quat().toRotationMatrix().eulerAngles(2, 1, 0);
        temp.pose.translation(
            {estimatedContact.pose.translation()[0], estimatedContact.pose.translation()[1], 0.0});
        temp.pose.quat(Eigen::AngleAxis(ypr(0), Eigen::Vector3d::UnitZ()));

        temp.type = Contacts::ContactType::FULL;
        contactListMap[footName].addContact(temp);
    }
    // In this case the contact ended so we have to set the deactivation time
    else if (contactListMap[footName].size() != 0 && contactListMap[footName].lastContact()->isContactActive(currentTime
                                                                     * this->slowDownFactor)
             && !estimatedContact.isActive)
    {
        Contacts::PlannedContact lastContact = *(contactListMap[footName].lastContact());
        lastContact.deactivationTime = currentTime * this->slowDownFactor;

        // we cannot update the status of the last contact. We need to remove it and add it again
        contactListMap[footName].removeLastContact();
        if (!contactListMap[footName].addContact(lastContact))
        {
            log()->error("[MANNTrajectoryGenerator::Impl::updateContactList] Unable to update the "
                         "contact list for the contact named '{}",
                         footName);
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

    bool ok = getParameter(paramHandler, "time_horizon", m_pimpl->horizon);
    ok = ok && getParameter(paramHandler, "sampling_time", m_pimpl->dT);
    ok = ok && getParameter(paramHandler, "slow_down_factor", m_pimpl->slowDownFactor);
    ok = ok
         && getParameter(paramHandler.lock()->getGroup("MANN"),
                         "projected_base_horizon",
                         m_pimpl->projectedBaseHorizonSize);
    m_pimpl->mergePointStates.resize(m_pimpl->horizon / m_pimpl->dT);
    m_pimpl->output.basePoses.resize(m_pimpl->horizon / m_pimpl->dT);
    m_pimpl->output.jointPositions.resize(m_pimpl->horizon / m_pimpl->dT);
    m_pimpl->output.angularMomentumTrajectory.resize(3, m_pimpl->horizon / m_pimpl->dT);
    m_pimpl->output.comTrajectory.resize(3, m_pimpl->horizon / m_pimpl->dT);

    return ok && m_pimpl->mann.initialize(paramHandler);
}

void MANNTrajectoryGenerator::setInitialState(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                                              const Contacts::EstimatedContact& leftFoot,
                                              const Contacts::EstimatedContact& rightFoot,
                                              const manif::SE3d& basePose,
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

    // TODO remove 51
    temp.autoregressiveState.pastProjectedBasePositions
        = std::deque<Eigen::Vector2d>{51, Eigen::Vector2d{0.0, 0.0}};
    temp.autoregressiveState.pastProjectedBaseVelocity
        = std::deque<Eigen::Vector2d>{51, Eigen::Vector2d{0.0, 0.0}};
    temp.autoregressiveState.pastFacingDirection
        = std::deque<Eigen::Vector2d>{51, Eigen::Vector2d{1.0, 0.0}};
    temp.autoregressiveState.I_H_FD = manif::SE2d::Identity();
    temp.autoregressiveState.previousMannInput = temp.input;

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
                     m_pimpl->horizon,
                     input.mergePointIndex * m_pimpl->dT);
        return false;
    }

    m_pimpl->mannAutoregressiveInput = input;

    const auto& mergePointState = m_pimpl->mergePointStates[input.mergePointIndex];
    if (!m_pimpl->mann.reset(mergePointState.input,
                             mergePointState.leftFoot,
                             mergePointState.rightFoot,
                             mergePointState.basePosition,
                             mergePointState.baseVelocity,
                             mergePointState.autoregressiveState,
                             mergePointState.time))
    {
        log()->error("{} Unable to reset MANN.");
        return false;
    }

    // clean the contact list map
    m_pimpl->contactListMap["left_foot"].clear();
    m_pimpl->contactListMap["right_foot"].clear();

    // add the first contact if needed
    m_pimpl->resetContactList(mergePointState.leftFoot, "left_foot");
    m_pimpl->resetContactList(mergePointState.rightFoot, "right_foot");

    return true;
}

bool MANNTrajectoryGenerator::advance()
{
    constexpr auto logPrefix = "[MANNTrajectoryGenerator::setInput]";

    // invalidate the output
    m_pimpl->isOutputValid = false;

    for (int i = 0; i < m_pimpl->mergePointStates.size(); i++)
    {
        if (!m_pimpl->mann.setInput(m_pimpl->mannAutoregressiveInput))
        {
            log()->error("{} Unable to set the input to MANN.");
            return false;
        }

        if (!m_pimpl->mann.advance())
        {
            log()->error("{} Unable to perform one iteration of MANN.");
            return false;
        }

        const auto& MANNOutput = m_pimpl->mann.getOutput();

        // store the status required to reset the system
        m_pimpl->mergePointStates[i].basePosition = MANNOutput.basePose;
        m_pimpl->mergePointStates[i].autoregressiveState = m_pimpl->mann.getAutoregressiveState();
        m_pimpl->mergePointStates[i].baseVelocity = MANNOutput.baseVelocity;
        m_pimpl->mergePointStates[i].input = m_pimpl->mann.getMANNInput();
        m_pimpl->mergePointStates[i].leftFoot = MANNOutput.leftFoot;
        m_pimpl->mergePointStates[i].rightFoot = MANNOutput.rightFoot;
        m_pimpl->mergePointStates[i].time = MANNOutput.currentTime;
        m_pimpl->output.basePoses[i] = MANNOutput.basePose;
        m_pimpl->output.jointPositions[i] = MANNOutput.jointsPosition;
        m_pimpl->output.angularMomentumTrajectory.col(i) = MANNOutput.angularMomentum;
        m_pimpl->output.comTrajectory.col(i) = MANNOutput.comPosition;

        if (!m_pimpl->updateContactList("left_foot", MANNOutput.currentTime, MANNOutput.leftFoot))
        {
            log()->error("{} Unable update the contact list for the left_foot.");
            return false;
        }

        if (!m_pimpl->updateContactList("right_foot", MANNOutput.currentTime, MANNOutput.rightFoot))
        {
            log()->error("{} Unable update the contact list for the right_foot.");
            return false;
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
