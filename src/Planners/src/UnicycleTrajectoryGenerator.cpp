/**
 * @file UnicycleTrajectoryGenerator.cpp
 * @authors Lorenzo Moretti, Giulio Romualdi, Stefano Dafarra
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Planners/UnicycleTrajectoryGenerator.h>
#include <BipedalLocomotion/Planners/UnicycleTrajectoryPlanner.h>
#include <BipedalLocomotion/Planners/UnicycleUtilities.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/Model.h>

#include <chrono>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using namespace BipedalLocomotion;

class Planners::UnicycleTrajectoryGenerator::Impl
{

public:
    enum class FSM
    {
        NotInitialized,
        Initialized,
        Running,
    };

    FSM state{FSM::NotInitialized};

    UnicycleTrajectoryGeneratorOutput output;

    UnicycleTrajectoryGeneratorInput input;

    UnicycleTrajectoryGeneratorParameters parameters;

    bool newTrajectoryRequired; /**< True if a new trajectory is required. False otherwise. */

    size_t newTrajectoryMergeCounter; /**< It is a counter that informs when the new
                                           trajectory will be merged.
                                           The new trajectory gets merged at the
                                           (newTrajectoryMergeCounter - 2) cycle. */

    std::chrono::nanoseconds time; /**< Current time */

    std::mutex mutex;

    enum class unicycleTrajectoryPlannerState
    {
        Called,
        Returned,
        Running,
    };

    unicycleTrajectoryPlannerState unicyclePlannerState{unicycleTrajectoryPlannerState::Returned};

    std::future<bool> unicyclePlannerOutputFuture;

    struct Trajectory
    {
        std::deque<Eigen::Vector2d> dcmPosition;
        std::deque<Eigen::Vector2d> dcmVelocity;
        std::deque<Eigen::Vector3d> comPosition;
        std::deque<Eigen::Vector3d> comVelocity;
        std::deque<Eigen::Vector3d> comAcceleration;
        std::deque<bool> leftFootinContact;
        std::deque<bool> rightFootinContact;
        std::deque<bool> isLeftFootLastSwinging;
        std::deque<size_t> mergePoints;
        std::deque<Step> leftSteps;
        std::deque<Step> rightSteps;
        std::deque<manif::SE3d> leftFootTransform;
        std::deque<manif::SE3d> rightFootTransform;
        std::deque<manif::SE3d::Tangent> leftFootMixedVelocity;
        std::deque<manif::SE3d::Tangent> rightFootMixedVelocity;
        std::deque<manif::SE3d::Tangent> leftFootMixedAcceleration;
        std::deque<manif::SE3d::Tangent> rightFootMixedAcceleration;
    };

    Trajectory trajectory;

    BipedalLocomotion::Planners::UnicycleTrajectoryPlanner unicycleTrajectoryPlanner;

    /**
     * ask for a new trajectory to the unicycle trajectory planner
     */
    bool askNewTrajectory(const std::chrono::nanoseconds& initTime,
                          const manif::SE3d& measuredTransform);

    /**
     * merge the current trajectory with the new one computed by the unicycle trajectory planner
     */
    bool mergeTrajectories(const size_t& mergePoint);

    /**
     * advance the current trajectory
     */
    bool advanceTrajectory();

    /*
     * @brief It resets <currentContactList> to an empty contact list. Then, if it exists, the
     * current active or the last active contact found in <previousContactList> is added to
     * <currentContactList>.
     * @param time The current time.
     * @param minStepDuration The minimum time duration of a step.
     * @param previousContactList The previous contact list, computed at last iteration.
     * @param currentContactList The current contact list, being generated.
     */
    static void resetContactList(const std::chrono::nanoseconds& time,
                                 const std::chrono::nanoseconds& minStepDuration,
                                 const Contacts::ContactList& previousContactList,
                                 Contacts::ContactList& currentContactList);
};

Planners::UnicycleTrajectoryGenerator::UnicycleTrajectoryGenerator()
{
    m_pImpl = std::make_unique<UnicycleTrajectoryGenerator::Impl>();
}

Planners::UnicycleTrajectoryGenerator::~UnicycleTrajectoryGenerator()
{

    Planners::UnicycleTrajectoryGenerator::Impl::unicycleTrajectoryPlannerState unicyclePlannerState;

    {
        std::lock_guard<std::mutex> lock(m_pImpl->mutex);
        unicyclePlannerState = m_pImpl->unicyclePlannerState;
    } // The mutex is automatically unlocked here

    if (unicyclePlannerState != Impl::unicycleTrajectoryPlannerState::Returned)
    {
        m_pImpl->unicyclePlannerOutputFuture.wait();
    }
}

Planners::UnicycleTrajectoryGeneratorInput
Planners::UnicycleTrajectoryGeneratorInput::generateDummyUnicycleTrajectoryGeneratorInput()
{
    UnicycleTrajectoryGeneratorInput input;

    input.plannerInput = Eigen::VectorXd::Zero(3);

    return input;
}

bool Planners::UnicycleTrajectoryGenerator::setRobotContactFrames(const iDynTree::Model& model)
{

    const auto logPrefix = "[UnicycleTrajectoryGenerator::setRobotContactFrames]";

    if (m_pImpl->state == Impl::FSM::NotInitialized)
    {
        log()->error("{} The Unicycle planner has not been initialized. Initialize it first.",
                     logPrefix);
        return false;
    }

    if (!m_pImpl->unicycleTrajectoryPlanner.setRobotContactFrames(model))
    {
        log()->error("{} Unable to set the robot contact frames.", logPrefix);
        m_pImpl->state = Impl::FSM::NotInitialized;
        return false;
    }

    m_pImpl->parameters.leftContactFrameIndex
        = m_pImpl->unicycleTrajectoryPlanner.getLeftContactFrameIndex();

    m_pImpl->parameters.rightContactFrameIndex
        = m_pImpl->unicycleTrajectoryPlanner.getRightContactFrameIndex();

    return true;
}

bool Planners::UnicycleTrajectoryGenerator::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[UnicycleTrajectoryGenerator::initialize]";

    bool ok{true};

    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return false;
    }

    // lambda function to parse parameters
    auto loadParam = [ptr, logPrefix](const std::string& paramName, auto& param) -> bool {
        if (!ptr->getParameter(paramName, param))
        {
            log()->error("{} Unable to get the parameter named '{}'.", logPrefix, paramName);
            return false;
        }
        return true;
    };

    // lambda function to parse parameters with fallback option
    auto loadParamWithFallback =
        [ptr, logPrefix](const std::string& paramName, auto& param, const auto& fallback) -> bool {
        if (!ptr->getParameter(paramName, param))
        {
            log()->info("{} Unable to find the parameter named '{}'. The default one with value "
                        "[{}] will be used.",
                        logPrefix,
                        paramName,
                        fallback);
            param = fallback;
        }
        return true;
    };

    double dt, plannerAdvanceTimeInS;
    ok = ok && loadParamWithFallback("dt", dt, 0.002);
    m_pImpl->parameters.dt = std::chrono::nanoseconds(static_cast<int64_t>(dt * 1e9));

    ok = ok && loadParamWithFallback("planner_advance_time_in_s", plannerAdvanceTimeInS, 0.18);

    m_pImpl->parameters.plannerAdvanceTimeSteps
        = std::round(plannerAdvanceTimeInS / dt) + 2; // The additional 2
                                                      // steps are because
                                                      // the trajectory from
                                                      // the planner is
                                                      // requested two steps
                                                      // in advance wrt the
                                                      // merge point

    // Initialize the time
    m_pImpl->time = std::chrono::nanoseconds::zero();

    // Initialize the merge points
    m_pImpl->trajectory.mergePoints.insert(m_pImpl->trajectory.mergePoints.begin(), 0);

    // Initialize the new trajectory merge counter
    m_pImpl->newTrajectoryMergeCounter = -1;

    // Initialize the new trajectory required flag
    m_pImpl->newTrajectoryRequired = false;

    // Initialize the blf unicycle planner
    ok = ok && m_pImpl->unicycleTrajectoryPlanner.initialize(ptr);

    // Initialize contact frames
    std::string leftContactFrameName, rightContactFrameName;
    ok = ok && loadParam("leftContactFrameName", leftContactFrameName);
    ok = ok && loadParam("rightContactFrameName", rightContactFrameName);

    // Initialize first trajectory
    ok = ok && generateFirstTrajectory();
    ok = ok && m_pImpl->mergeTrajectories(0);

    if (ok)
    {
        m_pImpl->state = Impl::FSM::Initialized;
    }

    return true;
}

const Planners::UnicycleTrajectoryGeneratorOutput&
Planners::UnicycleTrajectoryGenerator::getOutput() const
{
    constexpr auto logPrefix = "[UnicycleTrajectoryGenerator::getOutput]";

    Planners::UnicycleUtilities::populateVectorFromDeque(m_pImpl->trajectory.dcmPosition,
                                                         m_pImpl->output.dcmTrajectory.position);

    Planners::UnicycleUtilities::populateVectorFromDeque(m_pImpl->trajectory.dcmVelocity,
                                                         m_pImpl->output.dcmTrajectory.velocity);

    Planners::UnicycleUtilities::populateVectorFromDeque(m_pImpl->trajectory.comPosition,
                                                         m_pImpl->output.comTrajectory.position);

    Planners::UnicycleUtilities::populateVectorFromDeque(m_pImpl->trajectory.comVelocity,
                                                         m_pImpl->output.comTrajectory.velocity);

    Planners::UnicycleUtilities::populateVectorFromDeque(m_pImpl->trajectory.comAcceleration,
                                                         m_pImpl->output.comTrajectory.acceleration);

    Planners::UnicycleUtilities::populateVectorFromDeque(m_pImpl->trajectory.leftFootTransform,
                                                         m_pImpl->output.leftFootTrajectory
                                                             .transform);

    Planners::UnicycleUtilities::populateVectorFromDeque(m_pImpl->trajectory.rightFootTransform,
                                                         m_pImpl->output.rightFootTrajectory
                                                             .transform);

    Planners::UnicycleUtilities::populateVectorFromDeque(m_pImpl->trajectory.leftFootMixedVelocity,
                                                         m_pImpl->output.leftFootTrajectory
                                                             .mixedVelocity);

    Planners::UnicycleUtilities::populateVectorFromDeque(m_pImpl->trajectory.rightFootMixedVelocity,
                                                         m_pImpl->output.rightFootTrajectory
                                                             .mixedVelocity);

    Planners::UnicycleUtilities::populateVectorFromDeque(m_pImpl->trajectory
                                                             .leftFootMixedAcceleration,
                                                         m_pImpl->output.leftFootTrajectory
                                                             .mixedAcceleration);

    Planners::UnicycleUtilities::populateVectorFromDeque(m_pImpl->trajectory
                                                             .rightFootMixedAcceleration,
                                                         m_pImpl->output.rightFootTrajectory
                                                             .mixedAcceleration);

    // instatiate variables for the contact phase lists
    BipedalLocomotion::Contacts::ContactListMap contactListMap;
    BipedalLocomotion::Contacts::ContactList leftContactList, rightContactList;

    // reset the contact lists
    if (!m_pImpl->output.contactPhaseList.lists().empty())
    {
        m_pImpl->resetContactList(m_pImpl->time - m_pImpl->parameters.dt,
                                  m_pImpl->unicycleTrajectoryPlanner.getMinStepDuration(),
                                  m_pImpl->output.contactPhaseList.lists().at("left_foot"),
                                  leftContactList);
        m_pImpl->resetContactList(m_pImpl->time - m_pImpl->parameters.dt,
                                  m_pImpl->unicycleTrajectoryPlanner.getMinStepDuration(),
                                  m_pImpl->output.contactPhaseList.lists().at("right_foot"),
                                  rightContactList);
    }

    // get the left contact phase list
    std::vector<bool> leftFootInContact;
    Planners::UnicycleUtilities::populateVectorFromDeque(m_pImpl->trajectory.leftFootinContact,
                                                         leftFootInContact);

    if (!Planners::UnicycleUtilities::getContactList(m_pImpl->time,
                                                     m_pImpl->parameters.dt,
                                                     leftFootInContact,
                                                     m_pImpl->trajectory.leftSteps,
                                                     m_pImpl->parameters.leftContactFrameIndex,
                                                     "left_foot",
                                                     leftContactList))
    {

        log()->error("{} Unable to get the contact list for the left foot.", logPrefix);
        m_pImpl->output.isValid = false;
    };

    contactListMap["left_foot"] = leftContactList;

    // get the right contact phase list
    std::vector<bool> rightFootInContact;
    Planners::UnicycleUtilities::populateVectorFromDeque(m_pImpl->trajectory.rightFootinContact,
                                                         rightFootInContact);

    if (!Planners::UnicycleUtilities::getContactList(m_pImpl->time,
                                                     m_pImpl->parameters.dt,
                                                     rightFootInContact,
                                                     m_pImpl->trajectory.rightSteps,
                                                     m_pImpl->parameters.rightContactFrameIndex,
                                                     "right_foot",
                                                     rightContactList))
    {
        log()->error("{} Unable to get the contact list for the right foot.", logPrefix);
        m_pImpl->output.isValid = false;
    };

    contactListMap["right_foot"] = rightContactList;

    m_pImpl->output.contactPhaseList.setLists(contactListMap);

    return m_pImpl->output;
}

bool Planners::UnicycleTrajectoryGenerator::isOutputValid() const
{
    return (m_pImpl->state == Impl::FSM::Running) && (m_pImpl->output.isValid);
}

bool Planners::UnicycleTrajectoryGenerator::setInput(const UnicycleTrajectoryGeneratorInput& input)

{

    constexpr auto logPrefix = "[UnicycleTrajectoryGenerator::setInput]";

    if (m_pImpl->state == Impl::FSM::NotInitialized)
    {
        log()->error("{} The Unicycle planner has never been initialized.", logPrefix);
        return false;
    }

    // check if the input is valid
    if (input.plannerInput.size() > 3)
    {
        log()->error("{} The planner input has to be a vector of size 2 or 3.", logPrefix);
        return false;
    }

    m_pImpl->input = input;

    // the trajectory was already finished the new trajectory will be attached as soon as possible
    if (m_pImpl->trajectory.mergePoints.empty())
    {
        if (!(m_pImpl->trajectory.leftFootinContact.front()
              && m_pImpl->trajectory.rightFootinContact.front()))
        {
            log()->error("{} The trajectory has already finished but the system is not in double "
                         "support.",
                         logPrefix);
            return false;
        }

        if (m_pImpl->newTrajectoryRequired)
            return true;

        // Since the evaluation of a new trajectory takes time the new trajectory will be merged
        // after x cycles
        m_pImpl->newTrajectoryMergeCounter = m_pImpl->parameters.plannerAdvanceTimeSteps;
    }

    // the trajectory was not finished the new trajectory will be attached at the next merge point
    else
    {
        // Searches for the first merge point that is at least m_plannerAdvanceTimeSteps steps away
        auto firstMergePointAvailable
            = std::find_if(m_pImpl->trajectory.mergePoints.begin(),
                           m_pImpl->trajectory.mergePoints.end(),
                           [this](size_t input) {
                               return input >= this->m_pImpl->parameters.plannerAdvanceTimeSteps;
                           });

        if (firstMergePointAvailable != m_pImpl->trajectory.mergePoints.end())
        {
            if (m_pImpl->newTrajectoryRequired)
                return true;

            m_pImpl->newTrajectoryMergeCounter = *firstMergePointAvailable;
        } else
        {
            if (m_pImpl->newTrajectoryRequired)
                return true;

            m_pImpl->newTrajectoryMergeCounter = m_pImpl->parameters.plannerAdvanceTimeSteps;
        }
    }

    m_pImpl->newTrajectoryRequired = true;

    return true;
}

bool Planners::UnicycleTrajectoryGenerator::advance()
{
    constexpr auto logPrefix = "[UnicycleTrajectoryGenerator::advance]";

    if (m_pImpl->state == Impl::FSM::NotInitialized)
    {
        log()->error("{} The Unicycle Trajectory Generator has never been initialized.", logPrefix);
        return false;
    }

    // if a new trajectory is required check if its the time to evaluate the new trajectory or
    // the time to attach new one
    if (m_pImpl->newTrajectoryRequired)
    {
        // when we are near to the merge point the new trajectory is evaluated
        if (m_pImpl->newTrajectoryMergeCounter == m_pImpl->parameters.plannerAdvanceTimeSteps)
        {

            std::chrono::nanoseconds initTimeTrajectory
                = m_pImpl->time + m_pImpl->newTrajectoryMergeCounter * m_pImpl->parameters.dt;

            manif::SE3d measuredTransform = m_pImpl->trajectory.isLeftFootLastSwinging.front()
                                                ? m_pImpl->trajectory.rightFootTransform.at(
                                                    m_pImpl->newTrajectoryMergeCounter)
                                                : m_pImpl->trajectory.leftFootTransform.at(
                                                    m_pImpl->newTrajectoryMergeCounter);

            // ask for a new trajectory (and spawn an asynchronous thread to compute it)
            {
                std::lock_guard<std::mutex> lock(m_pImpl->mutex);

                if (m_pImpl->unicyclePlannerState == Impl::unicycleTrajectoryPlannerState::Running)
                {
                    log()->error("{} The unicycle planner is still running.", logPrefix);
                    return false;
                }
                m_pImpl->unicyclePlannerState = Impl::unicycleTrajectoryPlannerState::Called;
            } // The mutex is automatically unlocked here

            if (!m_pImpl->askNewTrajectory(initTimeTrajectory, measuredTransform))
            {
                log()->error("{} Unable to ask for a new trajectory.", logPrefix);
                return false;
            }
        }

        if (m_pImpl->newTrajectoryMergeCounter == 2)
        {
            if (!m_pImpl->mergeTrajectories(m_pImpl->newTrajectoryMergeCounter))
            {
                log()->error("{} Error while updating trajectories. They were not computed yet.",
                             logPrefix);

                return false;
            }
            m_pImpl->newTrajectoryRequired = false;
        }

        m_pImpl->newTrajectoryMergeCounter--;
    }

    // advance time
    m_pImpl->time += m_pImpl->parameters.dt;

    // advance the trajectory
    m_pImpl->advanceTrajectory();

    m_pImpl->state = Impl::FSM::Running;

    return true;
}

bool BipedalLocomotion::Planners::UnicycleTrajectoryGenerator::Impl::askNewTrajectory(
    const std::chrono::nanoseconds& initTime, const manif::SE3d& measuredTransform)
{
    constexpr auto logPrefix = "[UnicycleTrajectoryGenerator::Impl::askForNewTrajectory]";

    log()->debug("{} Asking for a new trajectory.", logPrefix);

    auto mergePoint = newTrajectoryMergeCounter;

    if (mergePoint >= trajectory.dcmPosition.size())
    {
        log()->error("{} The mergePoint has to be lower than the trajectory size.", logPrefix);
        return false;
    }

    // lambda function that computes the new trajectory
    auto computeNewTrajectory = [this]() -> bool {
        {
            std::lock_guard<std::mutex> lock(mutex);
            unicyclePlannerState = unicycleTrajectoryPlannerState::Running;
        } // The mutex is automatically unlocked here

        // advance the planner
        bool ok = this->unicycleTrajectoryPlanner.advance();

        {
            std::lock_guard<std::mutex> lock(mutex);
            unicyclePlannerState = unicycleTrajectoryPlannerState::Returned;
        } // The mutex is automatically unlocked here

        return ok;
    };

    // create the input for the unicycle planner
    UnicycleTrajectoryPlannerInput unicycleTrajectoryPlannerInput;
    unicycleTrajectoryPlannerInput.plannerInput = input.plannerInput;
    unicycleTrajectoryPlannerInput.initTime = initTime;
    unicycleTrajectoryPlannerInput.isLeftLastSwinging = trajectory.isLeftFootLastSwinging.front();
    unicycleTrajectoryPlannerInput.measuredTransform = measuredTransform;
    unicycleTrajectoryPlannerInput.dcmInitialState.initialPosition
        = trajectory.dcmPosition[mergePoint];
    unicycleTrajectoryPlannerInput.dcmInitialState.initialVelocity
        = trajectory.dcmVelocity[mergePoint];
    unicycleTrajectoryPlannerInput.comInitialState.initialPlanarPosition
        = trajectory.comPosition[mergePoint].head<2>();
    unicycleTrajectoryPlannerInput.comInitialState.initialPlanarVelocity
        = trajectory.comVelocity[mergePoint].head<2>();

    // set the input
    this->unicycleTrajectoryPlanner.setInput(unicycleTrajectoryPlannerInput);

    // create a new asynchronous thread to compute the new trajectory
    unicyclePlannerOutputFuture = std::async(std::launch::async, computeNewTrajectory);

    return true;
}

bool BipedalLocomotion::Planners::UnicycleTrajectoryGenerator::Impl::mergeTrajectories(
    const size_t& mergePoint)
{

    constexpr auto logPrefix = "[UnicycleTrajectoryGenerator::Impl::mergeTrajectories]";

    log()->debug("{} Merging trajectories at mergepoint {}", logPrefix, mergePoint);

    // if unicycle trajectory generator has been initialized, check if the unicycle planner has
    // finished the computation
    if (!(state == Impl::FSM::NotInitialized))
    {
        if (unicyclePlannerState != unicycleTrajectoryPlannerState::Returned)
        {
            log()->error("{} The unicycle planner is still running.", logPrefix);
            return false;
        }

        if (unicyclePlannerOutputFuture.valid())
        {
            if (!unicyclePlannerOutputFuture.get())
            {
                log()->error("{} Unable to advance the unicycle planner.", logPrefix);
                return false;
            }
        } else
        {
            log()->error("{} The trajectory is not valid at time {} [s].", logPrefix, time);
            return false;
        }
    }

    // get the output of the unicycle planner and append it to deques

    // get dcm position
    Planners::UnicycleUtilities::appendVectorToDeque(unicycleTrajectoryPlanner.getOutput()
                                                         .dcmTrajectory.position,
                                                     trajectory.dcmPosition,
                                                     mergePoint);
    // get dcm velocity
    Planners::UnicycleUtilities::appendVectorToDeque(unicycleTrajectoryPlanner.getOutput()
                                                         .dcmTrajectory.velocity,
                                                     trajectory.dcmVelocity,
                                                     mergePoint);
    // get feet contact status
    Planners::UnicycleUtilities::appendVectorToDeque(unicycleTrajectoryPlanner.getOutput()
                                                         .contactStatus.UsedLeftAsFixed,
                                                     trajectory.isLeftFootLastSwinging,
                                                     mergePoint);
    Planners::UnicycleUtilities::appendVectorToDeque(unicycleTrajectoryPlanner.getOutput()
                                                         .contactStatus.leftFootInContact,
                                                     trajectory.leftFootinContact,
                                                     mergePoint);
    Planners::UnicycleUtilities::appendVectorToDeque(unicycleTrajectoryPlanner.getOutput()
                                                         .contactStatus.rightFootInContact,
                                                     trajectory.rightFootinContact,
                                                     mergePoint);
    // get com trajectory
    Planners::UnicycleUtilities::appendVectorToDeque(unicycleTrajectoryPlanner.getOutput()
                                                         .comTrajectory.position,
                                                     trajectory.comPosition,
                                                     mergePoint);
    Planners::UnicycleUtilities::appendVectorToDeque(unicycleTrajectoryPlanner.getOutput()
                                                         .comTrajectory.velocity,
                                                     trajectory.comVelocity,
                                                     mergePoint);
    Planners::UnicycleUtilities::appendVectorToDeque(unicycleTrajectoryPlanner.getOutput()
                                                         .comTrajectory.acceleration,
                                                     trajectory.comAcceleration,
                                                     mergePoint);
    // get feet cubic spline trajectory

    // get left foot cubic spline
    Planners::UnicycleUtilities::appendVectorToDeque(unicycleTrajectoryPlanner.getOutput()
                                                         .leftFootTrajectory.transform,
                                                     trajectory.leftFootTransform,
                                                     mergePoint);
    Planners::UnicycleUtilities::appendVectorToDeque(unicycleTrajectoryPlanner.getOutput()
                                                         .leftFootTrajectory.mixedVelocity,
                                                     trajectory.leftFootMixedVelocity,
                                                     mergePoint);
    Planners::UnicycleUtilities::appendVectorToDeque(unicycleTrajectoryPlanner.getOutput()
                                                         .leftFootTrajectory.mixedAcceleration,
                                                     trajectory.leftFootMixedAcceleration,
                                                     mergePoint);

    // get right foot cubic spline
    Planners::UnicycleUtilities::appendVectorToDeque(unicycleTrajectoryPlanner.getOutput()
                                                         .rightFootTrajectory.transform,
                                                     trajectory.rightFootTransform,
                                                     mergePoint);
    Planners::UnicycleUtilities::appendVectorToDeque(unicycleTrajectoryPlanner.getOutput()
                                                         .rightFootTrajectory.mixedVelocity,
                                                     trajectory.rightFootMixedVelocity,
                                                     mergePoint);
    Planners::UnicycleUtilities::appendVectorToDeque(unicycleTrajectoryPlanner.getOutput()
                                                         .rightFootTrajectory.mixedAcceleration,
                                                     trajectory.rightFootMixedAcceleration,
                                                     mergePoint);

    // get steps
    auto newLeftSteps = unicycleTrajectoryPlanner.getOutput().steps.leftSteps;
    auto newRightSteps = unicycleTrajectoryPlanner.getOutput().steps.rightSteps;

    // merge steps
    BipedalLocomotion::Planners::UnicycleUtilities::mergeSteps(newLeftSteps,
                                                               trajectory.leftSteps,
                                                               time);
    BipedalLocomotion::Planners::UnicycleUtilities::mergeSteps(newRightSteps,
                                                               trajectory.rightSteps,
                                                               time);

    // get merge points
    std::vector<size_t> mergePoints;
    mergePoints = unicycleTrajectoryPlanner.getOutput().mergePoints;

    // update mergePoints of new trajectory based on where the new trajectory is merged
    for (auto& element : mergePoints)
    {
        element += mergePoint;
    }
    trajectory.mergePoints.assign(mergePoints.begin(), mergePoints.end());

    // pop the first merge point
    trajectory.mergePoints.pop_front();

    return true;
}

bool BipedalLocomotion::Planners::UnicycleTrajectoryGenerator::Impl::advanceTrajectory()
{

    constexpr auto logPrefix = "[UnicycleTrajectoryGenerator::Impl::advanceTrajectory]";

    // check if vector is not initialized
    if (trajectory.leftFootinContact.empty() || trajectory.rightFootinContact.empty()
        || trajectory.isLeftFootLastSwinging.empty() || trajectory.dcmPosition.empty()
        || trajectory.dcmVelocity.empty() || trajectory.comPosition.empty()
        || trajectory.comVelocity.empty() || trajectory.comAcceleration.empty())

    {
        log()->error(" {} Cannot advance empty trajectory signals.", logPrefix);
        return false;
    }

    // lambda function to advance the trajectory by 1 step
    auto trajectoryStepLambda = [](auto& inputDeque) {
        inputDeque.pop_front();
        inputDeque.push_back(inputDeque.back());
    };

    // advance trajectories
    bool rightWasInContact = trajectory.rightFootinContact.front();
    trajectoryStepLambda(trajectory.rightFootinContact);
    bool leftWasInContact = trajectory.leftFootinContact.front();
    trajectoryStepLambda(trajectory.leftFootinContact);
    trajectoryStepLambda(trajectory.isLeftFootLastSwinging);
    trajectoryStepLambda(trajectory.dcmPosition);
    trajectoryStepLambda(trajectory.dcmVelocity);
    trajectoryStepLambda(trajectory.comPosition);
    trajectoryStepLambda(trajectory.comVelocity);
    trajectoryStepLambda(trajectory.comAcceleration);
    trajectoryStepLambda(trajectory.leftFootTransform);
    trajectoryStepLambda(trajectory.leftFootMixedVelocity);
    trajectoryStepLambda(trajectory.leftFootMixedAcceleration);
    trajectoryStepLambda(trajectory.rightFootTransform);
    trajectoryStepLambda(trajectory.rightFootMixedVelocity);
    trajectoryStepLambda(trajectory.rightFootMixedAcceleration);

    // at each sampling time the merge points are decreased by one.
    // If the first merge point is equal to 0 it will be dropped.
    // A new trajectory will be merged at the first merge point or if the deque is empty
    // as soon as possible.
    if (!trajectory.mergePoints.empty())
    {
        for (auto& mergePoint : trajectory.mergePoints)
            mergePoint--;

        if (trajectory.mergePoints[0] == 0)
            trajectory.mergePoints.pop_front();
    }

    // if the left foot is leaving the contact, the step is dropped
    if (leftWasInContact && !trajectory.leftFootinContact.front())
    {
        trajectory.leftSteps.pop_front();
    }

    // if the right foot is leaving the contact, the step is dropped
    if (rightWasInContact && !trajectory.rightFootinContact.front())
    {
        trajectory.rightSteps.pop_front();
    }

    return true;
}

bool BipedalLocomotion::Planners::UnicycleTrajectoryGenerator::generateFirstTrajectory()
{

    constexpr auto logPrefix = "[UnicycleTrajectoryGenerator::generateFirstTrajectory]";

    UnicycleTrajectoryPlannerInput unicycleTrajectoryPlannerInput;
    unicycleTrajectoryPlannerInput.initTime = m_pImpl->time;
    unicycleTrajectoryPlannerInput.comInitialState.initialPlanarPosition = Eigen::Vector2d::Zero();
    unicycleTrajectoryPlannerInput.comInitialState.initialPlanarVelocity = Eigen::Vector2d::Zero();

    log()->debug("{} Generating the first trajectory.", logPrefix);

    if (!m_pImpl->unicycleTrajectoryPlanner.setInput(unicycleTrajectoryPlannerInput))
    {
        log()->error("{} Unable to set the input for the unicycle planner.", logPrefix);
        return false;
    }

    if (!m_pImpl->unicycleTrajectoryPlanner.advance())
    {
        log()->error("{} Unable to advance the unicycle planner.", logPrefix);
        return false;
    }

    return true;
}

void Planners::UnicycleTrajectoryGenerator::Impl::resetContactList(
    const std::chrono::nanoseconds& time,
    const std::chrono::nanoseconds& minStepDuration,
    const Contacts::ContactList& previousContactList,
    Contacts::ContactList& currentContactList)
{
    currentContactList.clear();

    if (previousContactList.size() == 0)
    {
        return;
    }

    auto activeContact = previousContactList.getActiveContact(time);

    // Is contact present at the current time?
    // If not, try to get the last active
    if (activeContact == previousContactList.end())
    {
        activeContact = previousContactList.getActiveContact(time - minStepDuration / 2);
    }

    // if any active contact found, add it to the current contact list
    if (!(activeContact == previousContactList.end()))
    {
        currentContactList.addContact(*activeContact);
    }
}

std::chrono::nanoseconds
BipedalLocomotion::Planners::UnicycleTrajectoryGenerator::getSamplingTime() const
{
    return m_pImpl->parameters.dt;
};
