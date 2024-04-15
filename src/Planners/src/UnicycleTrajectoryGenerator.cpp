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
#include <chrono>
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
        std::deque<iDynTree::Transform> leftFootPose;
        std::deque<iDynTree::Transform> leftFootinContact;
        std::deque<iDynTree::Transform> rightFootPose;
        std::deque<iDynTree::Transform> rightFootinContact;
        std::deque<bool> isLeftFootLastSwinging;
        std::deque<size_t> mergePoints; // The merge points of the trajectory.
    };

    ReferenceSignals referenceSignals;

    std::unique_ptr<BipedalLocomotion::Planners::UnicyclePlanner> unicyclePlanner;

    bool askNewTrajectory(const double& initTime, const iDynTree::Transform& measuredTransform);

    bool mergeTrajectories(const size_t& mergePoint);

    bool advanceTrajectory();
};

// BipedalLocomotion::Planners::UnicycleTrajectoryGeneratorInput BipedalLocomotion::Planners::
//     UnicycleTrajectoryGeneratorInput::generateDummyUnicycleTrajectoryGeneratorInput()
// {
//     UnicycleTrajectoryGeneratorInput input;

//     input.plannerInput = Eigen::VectorXd::Zero(3);

//     iDynTree::Vector2 dcmInitialPosition, dcmInitialVelocity;
//     dcmInitialPosition.zero();
//     dcmInitialVelocity.zero();
//     input.dcmInitialState.initialPosition = dcmInitialPosition;
//     input.dcmInitialState.initialVelocity = dcmInitialVelocity;

//     input.isLeftLastSwinging = false; // isLeftLastSwingingFoot (from previous planner run)

//     input.initTime = 0.0;

//     input.measuredTransform = iDynTree::Transform::Identity();
//     input.measuredTransform.setPosition(iDynTree::Position(0.0, -0.085, 0.0));

//     return input;
// }

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

            iDynTree::Transform measuredTransform
                = m_pImpl->referenceSignals.isLeftFootLastSwinging.front()
                      ? m_pImpl->referenceSignals.rightFootPose[m_pImpl->newTrajectoryMergeCounter]
                      : m_pImpl->referenceSignals.leftFootPose[m_pImpl->newTrajectoryMergeCounter];

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
        log()->error("{} The trajectory is not valid at time {}.", logPrefix, time);
        return false;
    }

    std::vector<iDynTree::Transform> leftTrajectory;
    std::vector<iDynTree::Transform> rightTrajectory;
    std::vector<iDynTree::Twist> leftTwistTrajectory;
    std::vector<iDynTree::Twist> rightTwistTrajectory;
    std::vector<Eigen::Vector2d> dcmPositionReference;
    std::vector<Eigen::Vector2d> dcmVelocityReference;
    std::vector<bool> rightInContact;
    std::vector<bool> leftInContact;
    std::vector<double> comHeightPositionReference;
    std::vector<double> comHeightVelocityReference;
    std::vector<double> comHeightAccelerationReference;
    std::vector<size_t> mergePoints;
    std::vector<bool> isLeftFixedFrame;
    std::vector<bool> isStancePhase;

    // get dcm position and velocity
    dcmPositionReference = unicyclePlanner->getOutput().dcmTrajectory.dcmPosition;
    dcmVelocityReference = unicyclePlanner->getOutput().dcmTrajectory.dcmVelocity;

    // get feet trajectories
    m_trajectoryGenerator->getFeetTrajectories(leftTrajectory, rightTrajectory);
    m_trajectoryGenerator->getFeetTwist(leftTwistTrajectory, rightTwistTrajectory);
    m_trajectoryGenerator->getFeetStandingPeriods(leftInContact, rightInContact);
    m_trajectoryGenerator->getWhenUseLeftAsFixed(isLeftFixedFrame);

    // get com height trajectory
    comHeightPositionReference = unicyclePlanner->getOutput().comHeightTrajectory.comHeightPosition;
    comHeightVelocityReference = unicyclePlanner->getOutput().comHeightTrajectory.comHeightVelocity;
    comHeightAccelerationReference
        = unicyclePlanner->getOutput().comHeightTrajectory.comHeightAcceleration;

    // get merge points
    mergePoints = unicyclePlanner->getOutput().mergePoints;

    // get stance phase flags
    m_trajectoryGenerator->getIsStancePhase(isStancePhase);

    // append vectors to deques
    Utilities::appendVectorToDeque(leftTrajectory, m_leftTrajectory, mergePoint);
    Utilities::appendVectorToDeque(rightTrajectory, m_rightTrajectory, mergePoint);
    Utilities::appendVectorToDeque(leftTwistTrajectory, m_leftTwistTrajectory, mergePoint);
    Utilities::appendVectorToDeque(rightTwistTrajectory, m_rightTwistTrajectory, mergePoint);
    Utilities::appendVectorToDeque(isLeftFixedFrame, m_isLeftFixedFrame, mergePoint);

    Utilities::appendVectorToDeque(dcmPositionReference, referenceSignals.dcmPosition, mergePoint);
    Utilities::appendVectorToDeque(dcmVelocityReference, referenceSignals.dcmVelocity, mergePoint);

    Utilities::appendVectorToDeque(leftInContact, m_leftInContact, mergePoint);
    Utilities::appendVectorToDeque(rightInContact, m_rightInContact, mergePoint);

    Utilities::appendVectorToDeque(comHeightPositionReference,
                                   referenceSignals.comHeightPosition,
                                   mergePoint);
    Utilities::appendVectorToDeque(comHeightVelocityReference,
                                   referenceSignals.comHeightVelocity,
                                   mergePoint);
    Utilities::appendVectorToDeque(comHeightAccelerationReference,
                                   referenceSignals.comHeightAcceleration,
                                   mergePoint);

    Utilities::appendVectorToDeque(isStancePhase, m_isStancePhase, mergePoint);

    referenceSignals.mergePoints.assign(mergePoints.begin(), mergePoints.end());

    // the first merge point is always equal to 0
    referenceSignals.mergePoints.pop_front();

    return true;
}

bool BipedalLocomotion::Planners::UnicycleTrajectoryGenerator::Impl::advanceTrajectory()
{
    // check if vector is not initialized
    if (m_leftTrajectory.empty() || m_rightTrajectory.empty() || m_leftInContact.empty()
        || m_rightInContact.empty() || m_DCMPositionDesired.empty() || m_DCMVelocityDesired.empty()
        || m_comHeightTrajectory.empty())
    {
        yError() << "[WalkingModule::advanceReferenceSignals] Cannot advance empty reference "
                    "signals.";
        return false;
    }

    m_rightTrajectory.pop_front();
    m_rightTrajectory.push_back(m_rightTrajectory.back());

    m_leftTrajectory.pop_front();
    m_leftTrajectory.push_back(m_leftTrajectory.back());

    m_rightTwistTrajectory.pop_front();
    m_rightTwistTrajectory.push_back(m_rightTwistTrajectory.back());

    m_leftTwistTrajectory.pop_front();
    m_leftTwistTrajectory.push_back(m_leftTwistTrajectory.back());

    m_rightInContact.pop_front();
    m_rightInContact.push_back(m_rightInContact.back());

    m_leftInContact.pop_front();
    m_leftInContact.push_back(m_leftInContact.back());

    m_isLeftFixedFrame.pop_front();
    m_isLeftFixedFrame.push_back(m_isLeftFixedFrame.back());

    m_DCMPositionDesired.pop_front();
    m_DCMPositionDesired.push_back(m_DCMPositionDesired.back());

    m_DCMVelocityDesired.pop_front();
    m_DCMVelocityDesired.push_back(m_DCMVelocityDesired.back());

    m_comHeightTrajectory.pop_front();
    m_comHeightTrajectory.push_back(m_comHeightTrajectory.back());

    m_comHeightVelocity.pop_front();
    m_comHeightVelocity.push_back(m_comHeightVelocity.back());

    m_isStancePhase.pop_front();
    m_isStancePhase.push_back(m_isStancePhase.back());

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