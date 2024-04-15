/**
 * @file UnicycleTrajectoryGenerator.cpp
 * @authors Lorenzo Moretti, Giulio Romualdi, Stefano Dafarra
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Planners/UnicyclePlanner.h>
#include <BipedalLocomotion/Planners/UnicycleTrajectoryGenerator.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <cmath>
#include <cstddef>
#include <deque>
#include <future>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/Position.h>
#include <iDynTree/Transform.h>
#include <yarp/os/RFModule.h>

#include <FootPrint.h>
#include <UnicycleGenerator.h>
#include <UnicyclePlanner.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/MatrixFixSize.h>
#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/Rotation.h>
#include <iDynTree/VectorFixSize.h>

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

    bool newTrajectoryRequired; // True if a new trajectory is required. False otherwise.

    size_t newTrajectoryMergeCounter; // The new trajectory will be merged after
    // (m_newTrajectoryMergeCounter - 2) cycles.

    double time; // The current time.

    std::future<bool> unicyclePlannerOutputFuture;

    struct ReferenceSignals
    {
        std::deque<Eigen::Vector2d> dcmPosition;
        std::deque<Eigen::Vector2d> dcmVelocity;
        std::deque<double> comHeightPosition;
        std::deque<double> comHeightVelocity;
        std::deque<double> comHeightAcceleration;
        std::deque<bool> leftFootinContact;
        std::deque<bool> rightFootinContact;
        std::deque<bool> isLeftFootLastSwinging;
        std::deque<size_t> mergePoints; // The merge points of the trajectory.
    };

    ReferenceSignals referenceSignals;

    std::unique_ptr<BipedalLocomotion::Planners::UnicyclePlanner> unicyclePlanner;

    bool askNewTrajectory(const double& initTime, const iDynTree::Transform& measuredTransform);

    bool mergeTrajectories(const size_t& mergePoint);

    bool advanceTrajectory();
};

Planners::UnicycleTrajectoryGenerator::UnicycleTrajectoryGenerator()
{
    m_pImpl = std::make_unique<UnicycleTrajectoryGenerator::Impl>();
}

Planners::UnicycleTrajectoryGenerator::~UnicycleTrajectoryGenerator() = default;

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

    ok = ok && loadParam("dt", m_pImpl->parameters.dt);

    double plannerAdvanceTimeInS;
    ok = ok && loadParam("planner_advance_time_in_s", plannerAdvanceTimeInS);
    m_pImpl->parameters.plannerAdvanceTimeSteps
        = std::round(plannerAdvanceTimeInS / m_pImpl->parameters.dt) + 2; // The additional 2
                                                                          // steps are because
                                                                          // the trajectory from
                                                                          // the planner is
                                                                          // requested two steps
                                                                          // in advance wrt the
                                                                          // merge point

    // Initialize the time
    m_pImpl->time = 0.0;

    // Initialize the merge points
    m_pImpl->referenceSignals.mergePoints.insert(m_pImpl->referenceSignals.mergePoints.begin(), 0);

    // Initialize the new trajectory merge counter
    m_pImpl->newTrajectoryMergeCounter = -1;

    // Initialize the new trajectory required flag
    m_pImpl->newTrajectoryRequired = false;

    // Initialize the blf unicycle planner
    ok = ok && m_pImpl->unicyclePlanner->initialize(handler);

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

    return m_pImpl->output;
}

bool Planners::UnicycleTrajectoryGenerator::isOutputValid() const
{
    return m_pImpl->state == Impl::FSM::Running;
}

bool Planners::UnicycleTrajectoryGenerator::setInput(const UnicycleTrajectoryGeneratorInput& input)

{

    // TO BE REMOVED, JUST FOR COMPILATION ========================================
    std::vector<bool> m_leftInContact;
    std::vector<bool> m_rightInContact;
    // ==============================================================================

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
    if (m_pImpl->referenceSignals.mergePoints.empty())
    {
        if (!(m_leftInContact.front() && m_rightInContact.front()))
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
            = std::find_if(m_pImpl->referenceSignals.mergePoints.begin(),
                           m_pImpl->referenceSignals.mergePoints.end(),
                           [this](size_t input) {
                               return input >= this->m_pImpl->parameters.plannerAdvanceTimeSteps;
                           });

        if (firstMergePointAvailable != m_pImpl->referenceSignals.mergePoints.end())
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

    bool isLeftFootLastSwinging{m_pImpl->referenceSignals.isLeftFootLastSwinging.front()};

    // if a new trajectory is required check if its the time to evaluate the new trajectory or
    // the time to attach new one
    if (m_pImpl->newTrajectoryRequired)
    {
        // when we are near to the merge point the new trajectory is evaluated
        if (m_pImpl->newTrajectoryMergeCounter == m_pImpl->parameters.plannerAdvanceTimeSteps)
        {

            double initTimeTrajectory;
            initTimeTrajectory
                = m_pImpl->time + m_pImpl->newTrajectoryMergeCounter * m_pImpl->parameters.dt;

            // check that both feet are in contact
            if (!(m_pImpl->referenceSignals.leftFootinContact.front())
                || (m_pImpl->referenceSignals.rightFootinContact.front()))
            {
                log()->error(" {} Unable to evaluate the new trajectory. "
                             "Both feet need to be in contact before and while computing a new "
                             "trajectory. Consider reducing planner_advance_time_in_s.",
                             logPrefix);
                return false;
            }

            iDynTree::Transform measuredTransform
                = m_pImpl->referenceSignals.isLeftFootLastSwinging.front()
                      ? m_pImpl->input.w_H_rightFoot
                      : m_pImpl->input.w_H_leftFoot;

            // ask for a new trajectory (and spawn an asynchronous thread to compute it)

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
    const double& initTime, const iDynTree::Transform& measuredTransform)
{
    constexpr auto logPrefix = "[UnicycleTrajectoryGenerator::Impl::askForNewTrajectory]";

    auto mergePoint = newTrajectoryMergeCounter;

    if (mergePoint >= referenceSignals.dcmPosition.size())
    {
        log()->error("{} The mergePoint has to be lower than the trajectory size.", logPrefix);
        return false;
    }

    // lambda function that computes the new trajectory
    auto computeNewTrajectory = [this]() -> bool {
        // advance the planner
        return this->unicyclePlanner->advance();
    };

    // create the input for the unicycle planner
    UnicyclePlannerInput unicyclePlannerInput;
    unicyclePlannerInput.plannerInput = input.plannerInput;
    unicyclePlannerInput.initTime = initTime;
    unicyclePlannerInput.isLeftLastSwinging = referenceSignals.isLeftFootLastSwinging.front();
    unicyclePlannerInput.measuredTransform = measuredTransform;
    unicyclePlannerInput.dcmInitialState.initialPosition = referenceSignals.dcmPosition[mergePoint];
    unicyclePlannerInput.dcmInitialState.initialVelocity = referenceSignals.dcmVelocity[mergePoint];

    // set the input
    this->unicyclePlanner->setInput(unicyclePlannerInput);

    // create a new asynchronous thread to compute the new trajectory
    unicyclePlannerOutputFuture = std::async(std::launch::async, computeNewTrajectory);

    return true;
}

bool BipedalLocomotion::Planners::UnicycleTrajectoryGenerator::Impl::mergeTrajectories(
    const size_t& mergePoint)
{

    constexpr auto logPrefix = "[UnicycleTrajectoryGenerator::Impl::mergeTrajectories]";

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

    std::vector<Eigen::Vector2d> dcmPositionReference;
    std::vector<Eigen::Vector2d> dcmVelocityReference;
    std::vector<double> comHeightPositionReference;
    std::vector<double> comHeightVelocityReference;
    std::vector<double> comHeightAccelerationReference;
    std::vector<bool> rightInContact;
    std::vector<bool> leftInContact;
    std::vector<bool> isLastSwingingFoot;
    std::vector<size_t> mergePoints;

    // get dcm position and velocity
    dcmPositionReference = unicyclePlanner->getOutput().dcmTrajectory.dcmPosition;
    dcmVelocityReference = unicyclePlanner->getOutput().dcmTrajectory.dcmVelocity;

    // get com height trajectory
    comHeightPositionReference = unicyclePlanner->getOutput().comHeightTrajectory.comHeightPosition;
    comHeightVelocityReference = unicyclePlanner->getOutput().comHeightTrajectory.comHeightVelocity;
    comHeightAccelerationReference
        = unicyclePlanner->getOutput().comHeightTrajectory.comHeightAcceleration;

    // get feet contact status
    leftInContact = unicyclePlanner->getOutput().contactStatus.leftFootInContact;
    rightInContact = unicyclePlanner->getOutput().contactStatus.rightFootInContact;
    isLastSwingingFoot = unicyclePlanner->getOutput().contactStatus.UsedLeftAsFixed;

    // get merge points
    mergePoints = unicyclePlanner->getOutput().mergePoints;

    // append vectors to deques

    Utilities::appendVectorToDeque(isLastSwingingFoot,
                                   referenceSignals.isLeftFootLastSwinging,
                                   mergePoint);

    Utilities::appendVectorToDeque(dcmPositionReference, referenceSignals.dcmPosition, mergePoint);
    Utilities::appendVectorToDeque(dcmVelocityReference, referenceSignals.dcmVelocity, mergePoint);

    Utilities::appendVectorToDeque(leftInContact, referenceSignals.leftFootinContact, mergePoint);
    Utilities::appendVectorToDeque(rightInContact, referenceSignals.rightFootinContact, mergePoint);

    Utilities::appendVectorToDeque(comHeightPositionReference,
                                   referenceSignals.comHeightPosition,
                                   mergePoint);
    Utilities::appendVectorToDeque(comHeightVelocityReference,
                                   referenceSignals.comHeightVelocity,
                                   mergePoint);
    Utilities::appendVectorToDeque(comHeightAccelerationReference,
                                   referenceSignals.comHeightAcceleration,
                                   mergePoint);

    referenceSignals.mergePoints.assign(mergePoints.begin(), mergePoints.end());

    // the first merge point is always equal to 0
    referenceSignals.mergePoints.pop_front();

    return true;
}

bool BipedalLocomotion::Planners::UnicycleTrajectoryGenerator::Impl::advanceTrajectory()
{

    constexpr auto logPrefix = "[UnicycleTrajectoryGenerator::Impl::advanceTrajectory]";

    // check if vector is not initialized
    if (referenceSignals.leftFootinContact.empty() || referenceSignals.rightFootinContact.empty()
        || referenceSignals.isLeftFootLastSwinging.empty() || referenceSignals.dcmPosition.empty()
        || referenceSignals.dcmVelocity.empty() || referenceSignals.comHeightPosition.empty()
        || referenceSignals.comHeightVelocity.empty()
        || referenceSignals.comHeightAcceleration.empty())

    {
        log()->error(" {} Cannot advance empty reference signals.", logPrefix);
        return false;
    }

    referenceSignals.rightFootinContact.pop_front();
    referenceSignals.rightFootinContact.push_back(referenceSignals.rightFootinContact.back());

    referenceSignals.leftFootinContact.pop_front();
    referenceSignals.leftFootinContact.push_back(referenceSignals.leftFootinContact.back());

    referenceSignals.isLeftFootLastSwinging.pop_front();
    referenceSignals.isLeftFootLastSwinging.push_back(
        referenceSignals.isLeftFootLastSwinging.back());

    referenceSignals.dcmPosition.pop_front();
    referenceSignals.dcmPosition.push_back(referenceSignals.dcmPosition.back());

    referenceSignals.dcmVelocity.pop_front();
    referenceSignals.dcmVelocity.push_back(referenceSignals.dcmVelocity.back());

    referenceSignals.comHeightPosition.pop_front();
    referenceSignals.comHeightPosition.push_back(referenceSignals.comHeightPosition.back());

    referenceSignals.comHeightVelocity.pop_front();
    referenceSignals.comHeightVelocity.push_back(referenceSignals.comHeightVelocity.back());

    referenceSignals.comHeightAcceleration.pop_front();
    referenceSignals.comHeightAcceleration.push_back(referenceSignals.comHeightAcceleration.back());

    // at each sampling time the merge points are decreased by one.
    // If the first merge point is equal to 0 it will be dropped.
    // A new trajectory will be merged at the first merge point or if the deque is empty
    // as soon as possible.
    if (!referenceSignals.mergePoints.empty())
    {
        for (auto& mergePoint : referenceSignals.mergePoints)
            mergePoint--;

        if (referenceSignals.mergePoints[0] == 0)
            referenceSignals.mergePoints.pop_front();
    }
    return true;
}