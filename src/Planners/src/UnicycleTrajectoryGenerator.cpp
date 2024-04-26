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

#include <StepPhase.h>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <future>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/Position.h>
#include <iDynTree/Transform.h>
#include <memory>
#include <mutex>
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

    std::mutex mutex;

    enum class uniCyclePlannerState
    {
        Called,
        Returned,
        Running,
    };

    uniCyclePlannerState unicyclePlannerState{uniCyclePlannerState::Returned};

    std::future<bool> unicyclePlannerOutputFuture;

    struct ReferenceSignals
    {
        std::deque<Eigen::Vector2d> dcmPosition;
        std::deque<Eigen::Vector2d> dcmVelocity;
        std::deque<double> comHeightPosition;
        std::deque<double> comHeightVelocity;
        std::deque<double> comHeightAcceleration;
        std::deque<Eigen::Vector2d> comPlanarPosition;
        std::deque<Eigen::Vector2d> comPlanarVelocity;
        std::deque<Eigen::Vector2d> comPlanarAcceleration;
        std::deque<bool> leftFootinContact;
        std::deque<bool> rightFootinContact;
        std::deque<bool> isLeftFootLastSwinging;
        std::deque<size_t> mergePoints;
        std::deque<StepPhase> leftStepPhases;
        std::deque<StepPhase> rightStepPhases;
        std::deque<Step> leftSteps;
        std::deque<Step> rightSteps;
    };

    ReferenceSignals referenceSignals;

    BipedalLocomotion::Planners::UnicyclePlanner unicyclePlanner;

    bool askNewTrajectory(const double& initTime, const iDynTree::Transform& measuredTransform);

    bool mergeTrajectories(const size_t& mergePoint);

    bool advanceTrajectory();

    /*
     * @brief It resets <currentContactList> to an empty contact list. Then, if it exists, the
     * current active contact found in <previousContactList> is added to <currentContactList>.
     * @param time The current time.
     * @param previousContactList The previous contact list, computed at last iteration.
     * @param currentContactList The current contact list, being generated.
     */
    static void resetContactList(const double& time,
                                 const Contacts::ContactList& previousContactList,
                                 Contacts::ContactList& currentContactList);
};

Planners::UnicycleTrajectoryGenerator::UnicycleTrajectoryGenerator()
{
    m_pImpl = std::make_unique<UnicycleTrajectoryGenerator::Impl>();
}

Planners::UnicycleTrajectoryGenerator::~UnicycleTrajectoryGenerator()
{
    m_pImpl->mutex.lock();
    auto unicyclePlannerState = m_pImpl->unicyclePlannerState;
    m_pImpl->mutex.unlock();

    if (m_pImpl->unicyclePlannerState != Impl::uniCyclePlannerState::Returned)
    {
        m_pImpl->unicyclePlannerOutputFuture.wait();
    }
}

Planners::UnicycleTrajectoryGeneratorInput
Planners::UnicycleTrajectoryGeneratorInput::generateDummyUnicycleTrajectoryGeneratorInput()
{
    UnicycleTrajectoryGeneratorInput input;

    input.plannerInput = Eigen::VectorXd::Zero(3);

    input.w_H_leftFoot = iDynTree::Transform::Identity();
    input.w_H_leftFoot.setPosition({0.0, 0.1, 0.0});

    input.w_H_rightFoot = iDynTree::Transform::Identity();
    input.w_H_rightFoot.setPosition({0.0, -0.1, 0.0});

    return input;
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

    ok = ok && loadParamWithFallback("dt", m_pImpl->parameters.dt, 0.002);

    double plannerAdvanceTimeInS;
    ok = ok && loadParamWithFallback("planner_advance_time_in_s", plannerAdvanceTimeInS, 0.08);
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
    ok = ok && m_pImpl->unicyclePlanner.initialize(ptr);

    // Initialize contact frames
    std::string leftContactFrameName, rightContactFrameName;
    ok = ok && loadParam("leftContactFrameName", leftContactFrameName);
    ok = ok && loadParam("rightContactFrameName", rightContactFrameName);

    std::string modelPath
        = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName("model.urdf");
    BipedalLocomotion::log()->info("{} Model path: {}", logPrefix, modelPath);

    iDynTree::ModelLoader ml;
    if (!ml.loadModelFromFile(modelPath))
    {
        log()->error("{} Unable to load the model.urdf from {}", logPrefix, modelPath);
        return false;
    }

    auto tmpKinDyn = std::make_shared<iDynTree::KinDynComputations>();
    tmpKinDyn->loadRobotModel(ml.model());

    m_pImpl->parameters.leftContactFrameIndex
        = tmpKinDyn->model().getFrameIndex(leftContactFrameName);
    if (m_pImpl->parameters.leftContactFrameIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} Unable to find the frame named {}.", logPrefix, leftContactFrameName);
        return false;
    }

    m_pImpl->parameters.rightContactFrameIndex
        = tmpKinDyn->model().getFrameIndex(rightContactFrameName);
    if (m_pImpl->parameters.rightContactFrameIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} Unable to find the frame named {}.", logPrefix, rightContactFrameName);
        return false;
    }

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

    Planners::Utilities::populateVectorFromDeque(m_pImpl->referenceSignals.dcmPosition,
                                                 m_pImpl->output.dcmTrajectory.dcmPosition);

    Planners::Utilities::populateVectorFromDeque(m_pImpl->referenceSignals.dcmVelocity,
                                                 m_pImpl->output.dcmTrajectory.dcmVelocity);

    Planners::Utilities::populateVectorFromDeque(m_pImpl->referenceSignals.comHeightPosition,
                                                 m_pImpl->output.comHeightTrajectory
                                                     .comHeightPosition);

    Planners::Utilities::populateVectorFromDeque(m_pImpl->referenceSignals.comPlanarPosition,
                                                 m_pImpl->output.comPlanarTrajectory
                                                     .comPlanarPosition);

    Planners::Utilities::populateVectorFromDeque(m_pImpl->referenceSignals.comPlanarVelocity,
                                                 m_pImpl->output.comPlanarTrajectory
                                                     .comPlanarVelocity);

    Planners::Utilities::populateVectorFromDeque(m_pImpl->referenceSignals.comPlanarAcceleration,
                                                 m_pImpl->output.comPlanarTrajectory
                                                     .comPlanarAcceleration);

    Planners::Utilities::populateVectorFromDeque(m_pImpl->referenceSignals.comHeightVelocity,
                                                 m_pImpl->output.comHeightTrajectory
                                                     .comHeightVelocity);

    Planners::Utilities::populateVectorFromDeque(m_pImpl->referenceSignals.comHeightAcceleration,
                                                 m_pImpl->output.comHeightTrajectory
                                                     .comHeightAcceleration);
    // instatiate variables for the contact phase lists
    BipedalLocomotion::Contacts::ContactListMap contactListMap;
    BipedalLocomotion::Contacts::ContactList leftContactList, rightContactList;

    // reset the contact lists
    if (!m_pImpl->output.ContactPhaseList.lists().empty())
    {
        m_pImpl->resetContactList(m_pImpl->time - m_pImpl->parameters.dt,
                                  m_pImpl->output.ContactPhaseList.lists().at("left_foot"),
                                  leftContactList);
        m_pImpl->resetContactList(m_pImpl->time - m_pImpl->parameters.dt,
                                  m_pImpl->output.ContactPhaseList.lists().at("right_foot"),
                                  rightContactList);
    }

    // get the lift contact phase list
    std::vector<StepPhase> leftStepPhases;
    Planners::Utilities::populateVectorFromDeque(m_pImpl->referenceSignals.leftStepPhases,
                                                 leftStepPhases);

    std::vector<bool> leftFootInContact;
    Planners::Utilities::populateVectorFromDeque(m_pImpl->referenceSignals.leftFootinContact,
                                                 leftFootInContact);

    if (!Planners::Utilities::getContactList(m_pImpl->time,
                                             m_pImpl->parameters.dt,
                                             leftFootInContact,
                                             m_pImpl->referenceSignals.leftSteps,
                                             m_pImpl->parameters.leftContactFrameIndex,
                                             "left_foot",
                                             leftContactList))
    {

        log()->error("{} Unable to get the contact list for the left foot.", logPrefix);
        m_pImpl->output.isValid = false;
    };

    contactListMap["left_foot"] = leftContactList;

    // get the right contact phase list
    std::vector<StepPhase> rightStepPhases;
    Planners::Utilities::populateVectorFromDeque(m_pImpl->referenceSignals.rightStepPhases,
                                                 rightStepPhases);

    std::vector<bool> rightFootInContact;
    Planners::Utilities::populateVectorFromDeque(m_pImpl->referenceSignals.rightFootinContact,
                                                 rightFootInContact);

    if (!Planners::Utilities::getContactList(m_pImpl->time,
                                             m_pImpl->parameters.dt,
                                             rightFootInContact,
                                             m_pImpl->referenceSignals.rightSteps,
                                             m_pImpl->parameters.rightContactFrameIndex,
                                             "right_foot",
                                             rightContactList))
    {
        log()->error("{} Unable to get the contact list for the right foot.", logPrefix);
        m_pImpl->output.isValid = false;
    };

    contactListMap["right_foot"] = rightContactList;

    m_pImpl->output.ContactPhaseList.setLists(contactListMap);

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
    if (m_pImpl->referenceSignals.mergePoints.empty())
    {
        if (!(m_pImpl->referenceSignals.leftFootinContact.front()
              && m_pImpl->referenceSignals.rightFootinContact.front()))
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
                || !(m_pImpl->referenceSignals.rightFootinContact.front()))
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
            m_pImpl->mutex.lock();
            if (m_pImpl->unicyclePlannerState == Impl::uniCyclePlannerState::Running)
            {
                log()->error("{} The unicycle planner is still running.", logPrefix);
                return false;
            }
            m_pImpl->unicyclePlannerState = Impl::uniCyclePlannerState::Called;
            m_pImpl->mutex.unlock();

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

    log()->debug("{} Asking for a new trajectory.", logPrefix);

    auto mergePoint = newTrajectoryMergeCounter;

    if (mergePoint >= referenceSignals.dcmPosition.size())
    {
        log()->error("{} The mergePoint has to be lower than the trajectory size.", logPrefix);
        return false;
    }

    // lambda function that computes the new trajectory
    auto computeNewTrajectory = [this]() -> bool {
        mutex.lock();
        unicyclePlannerState = uniCyclePlannerState::Running;
        mutex.unlock();

        // advance the planner
        bool ok = this->unicyclePlanner.advance();

        mutex.lock();
        unicyclePlannerState = uniCyclePlannerState::Returned;
        mutex.unlock();

        return ok;
    };

    // create the input for the unicycle planner
    UnicyclePlannerInput unicyclePlannerInput;
    unicyclePlannerInput.plannerInput = input.plannerInput;
    unicyclePlannerInput.initTime = initTime;
    unicyclePlannerInput.isLeftLastSwinging = referenceSignals.isLeftFootLastSwinging.front();
    unicyclePlannerInput.measuredTransform = measuredTransform;
    unicyclePlannerInput.dcmInitialState.initialPosition = referenceSignals.dcmPosition[mergePoint];
    unicyclePlannerInput.dcmInitialState.initialVelocity = referenceSignals.dcmVelocity[mergePoint];
    unicyclePlannerInput.comInitialState.initialPlanarPosition
        = referenceSignals.comPlanarPosition[mergePoint];
    unicyclePlannerInput.comInitialState.initialPlanarVelocity
        = referenceSignals.comPlanarVelocity[mergePoint];

    // set the input
    this->unicyclePlanner.setInput(unicyclePlannerInput);

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
        if (unicyclePlannerState != uniCyclePlannerState::Returned)
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

    // get the output of the unicycle planner
    std::vector<Eigen::Vector2d> dcmPositionReference, dcmVelocityReference,
        comPlanarPositionReference, comPlanarVelocityReference, comPlanarAccelerationReference;
    std::vector<double> comHeightPositionReference, comHeightVelocityReference,
        comHeightAccelerationReference;
    std::vector<bool> rightInContact, leftInContact, isLastSwingingFoot;
    std::vector<size_t> mergePoints;
    std::vector<StepPhase> leftStepPhases, rightStepPhases;
    std::deque<Step> leftSteps, rightSteps;

    // get dcm position and velocity
    dcmPositionReference = unicyclePlanner.getOutput().dcmTrajectory.dcmPosition;
    dcmVelocityReference = unicyclePlanner.getOutput().dcmTrajectory.dcmVelocity;

    // get com height trajectory
    comHeightPositionReference = unicyclePlanner.getOutput().comHeightTrajectory.comHeightPosition;
    comHeightVelocityReference = unicyclePlanner.getOutput().comHeightTrajectory.comHeightVelocity;
    comHeightAccelerationReference
        = unicyclePlanner.getOutput().comHeightTrajectory.comHeightAcceleration;

    // get com planar trajectory
    comPlanarPositionReference = unicyclePlanner.getOutput().comPlanarTrajectory.comPlanarPosition;
    comPlanarVelocityReference = unicyclePlanner.getOutput().comPlanarTrajectory.comPlanarVelocity;
    comPlanarAccelerationReference
        = unicyclePlanner.getOutput().comPlanarTrajectory.comPlanarAcceleration;

    // get feet contact status
    leftInContact = unicyclePlanner.getOutput().contactStatus.leftFootInContact;
    rightInContact = unicyclePlanner.getOutput().contactStatus.rightFootInContact;
    isLastSwingingFoot = unicyclePlanner.getOutput().contactStatus.UsedLeftAsFixed;

    // get merge points
    mergePoints = unicyclePlanner.getOutput().mergePoints;

    // get steps
    leftSteps = unicyclePlanner.getOutput().steps.leftSteps;
    rightSteps = unicyclePlanner.getOutput().steps.rightSteps;

    // get step phases
    leftStepPhases = unicyclePlanner.getOutput().steps.leftStepPhases;
    rightStepPhases = unicyclePlanner.getOutput().steps.rightStepPhases;

    // append vectors to deques

    Planners::Utilities::appendVectorToDeque(isLastSwingingFoot,
                                             referenceSignals.isLeftFootLastSwinging,
                                             mergePoint);

    Planners::Utilities::appendVectorToDeque(dcmPositionReference,
                                             referenceSignals.dcmPosition,
                                             mergePoint);
    Planners::Utilities::appendVectorToDeque(dcmVelocityReference,
                                             referenceSignals.dcmVelocity,
                                             mergePoint);

    Planners::Utilities::appendVectorToDeque(leftInContact,
                                             referenceSignals.leftFootinContact,
                                             mergePoint);
    Planners::Utilities::appendVectorToDeque(rightInContact,
                                             referenceSignals.rightFootinContact,
                                             mergePoint);

    Planners::Utilities::appendVectorToDeque(comHeightPositionReference,
                                             referenceSignals.comHeightPosition,
                                             mergePoint);
    Planners::Utilities::appendVectorToDeque(comHeightVelocityReference,
                                             referenceSignals.comHeightVelocity,
                                             mergePoint);
    Planners::Utilities::appendVectorToDeque(comHeightAccelerationReference,
                                             referenceSignals.comHeightAcceleration,
                                             mergePoint);
    Planners::Utilities::appendVectorToDeque(comPlanarPositionReference,
                                             referenceSignals.comPlanarPosition,
                                             mergePoint);
    Planners::Utilities::appendVectorToDeque(comPlanarVelocityReference,
                                             referenceSignals.comPlanarVelocity,
                                             mergePoint);
    Planners::Utilities::appendVectorToDeque(comPlanarAccelerationReference,
                                             referenceSignals.comPlanarAcceleration,
                                             mergePoint);

    Planners::Utilities::appendVectorToDeque(leftStepPhases,
                                             referenceSignals.leftStepPhases,
                                             mergePoint);
    Planners::Utilities::appendVectorToDeque(rightStepPhases,
                                             referenceSignals.rightStepPhases,
                                             mergePoint);

    referenceSignals.leftSteps.assign(leftSteps.begin(), leftSteps.end());
    referenceSignals.rightSteps.assign(rightSteps.begin(), rightSteps.end());

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
        || referenceSignals.comHeightAcceleration.empty() || referenceSignals.leftStepPhases.empty()
        || referenceSignals.rightStepPhases.empty() || referenceSignals.comPlanarPosition.empty())

    {
        log()->error(" {} Cannot advance empty reference signals.", logPrefix);
        return false;
    }

    bool rightWasInContact = referenceSignals.rightFootinContact.front();
    referenceSignals.rightFootinContact.pop_front();
    referenceSignals.rightFootinContact.push_back(referenceSignals.rightFootinContact.back());

    bool leftWasInContact = referenceSignals.leftFootinContact.front();
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

    referenceSignals.comPlanarPosition.pop_front();
    referenceSignals.comPlanarPosition.push_back(referenceSignals.comPlanarPosition.back());

    referenceSignals.comPlanarVelocity.pop_front();
    referenceSignals.comPlanarVelocity.push_back(referenceSignals.comPlanarVelocity.back());

    referenceSignals.comPlanarAcceleration.pop_front();
    referenceSignals.comPlanarAcceleration.push_back(referenceSignals.comPlanarAcceleration.back());

    referenceSignals.comHeightAcceleration.pop_front();
    referenceSignals.comHeightAcceleration.push_back(referenceSignals.comHeightAcceleration.back());

    referenceSignals.leftStepPhases.pop_front();
    referenceSignals.leftStepPhases.push_back(referenceSignals.leftStepPhases.back());

    referenceSignals.rightStepPhases.pop_front();
    referenceSignals.rightStepPhases.push_back(referenceSignals.rightStepPhases.back());

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

    // if the left foot is leaving the contact, the step is dropped
    if (leftWasInContact && !referenceSignals.leftFootinContact.front())
    {
        referenceSignals.leftSteps.pop_front();
    }

    // if the right foot is leaving the contact, the step is dropped
    if (rightWasInContact && !referenceSignals.rightFootinContact.front())
    {
        referenceSignals.rightSteps.pop_front();
    }

    return true;
}

bool BipedalLocomotion::Planners::UnicycleTrajectoryGenerator::generateFirstTrajectory()
{

    constexpr auto logPrefix = "[UnicycleTrajectoryGenerator::generateFirstTrajectory]";

    UnicyclePlannerInput unicyclePlannerInput;
    unicyclePlannerInput.initTime = m_pImpl->time;
    unicyclePlannerInput.comInitialState.initialPlanarPosition = Eigen::Vector2d::Zero();
    unicyclePlannerInput.comInitialState.initialPlanarVelocity = Eigen::Vector2d::Zero();

    log()->debug("{} Generating the first trajectory.", logPrefix);

    if (!m_pImpl->unicyclePlanner.setInput(unicyclePlannerInput))
    {
        log()->error("{} Unable to set the input for the unicycle planner.", logPrefix);
        return false;
    }

    if (!m_pImpl->unicyclePlanner.advance())
    {
        log()->error("{} Unable to advance the unicycle planner.", logPrefix);
        return false;
    }

    return true;
}

void Planners::UnicycleTrajectoryGenerator::Impl::resetContactList(
    const double& time,
    const Contacts::ContactList& previousContactList,
    Contacts::ContactList& currentContactList)
{
    currentContactList.clear();

    if (previousContactList.size() == 0)
    {
        return;
    }

    auto presentContact = previousContactList.getPresentContact(
        std::chrono::milliseconds(static_cast<int>(time * 1000)));

    // Is contact present at the current time?
    if (!(presentContact == previousContactList.end()))
    {
        currentContactList.addContact(*presentContact);
    }
}