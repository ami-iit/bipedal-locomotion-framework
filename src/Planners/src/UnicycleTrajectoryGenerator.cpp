/**
 * @file UnicycleTrajectoryGenerator.cpp
 * @authors Lorenzo Moretti, Giulio Romualdi, Stefano Dafarra
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Planners/UnicycleTrajectoryGenerator.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <cmath>
#include <cstddef>
#include <iDynTree/Position.h>
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

    bool newTrajectoryRequired; // True if a new trajectory is required. False otherwise.

    std::deque<size_t> mergePoints; // The merge points of the trajectory.

    size_t newTrajectoryMergeCounter; // The new trajectory will be merged after
    // (m_newTrajectoryMergeCounter - 2) cycles.

    size_t m_plannerAdvanceTimeSteps; //++++++++ MAYBE TO BE REMOVED FROM HERE

    struct ReferenceSignals
    {
        Eigen::Vector3d dcmPosition;
        Eigen::Vector3d dcmVelocity;
        Eigen::Vector3d dcmAcceleration;
        Eigen::Vector3d comPosition;
        Eigen::Vector3d comVelocity;
        Eigen::Vector3d comAcceleration;
    };

    std::unique_ptr<BipedalLocomotion::Planners::UnicyclePlanner> unicyclePlanner;

    bool askForNewTrajectory();
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

    // Initialize the blf unicycle planner
    ok = ok && m_pImpl->unicyclePlanner->initialize(handler);

    // Initialize the merge points
    m_pImpl->mergePoints.insert(m_pImpl->mergePoints.begin(), 0);

    // Initialize the new trajectory merge counter
    m_pImpl->newTrajectoryMergeCounter = -1;

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

    m_pImpl->input = input;

    // the trajectory was already finished the new trajectory will be attached as soon as possible
    if (m_pImpl->mergePoints.empty())
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
        m_pImpl->newTrajectoryMergeCounter = m_pImpl->m_plannerAdvanceTimeSteps;
    }

    // the trajectory was not finished the new trajectory will be attached at the next merge point
    else
    {
        // Searches for the first merge point that is at least m_plannerAdvanceTimeSteps steps away
        auto firstMergePointAvailable
            = std::find_if(m_pImpl->mergePoints.begin(),
                           m_pImpl->mergePoints.end(),
                           [this](size_t input) {
                               return input >= this->m_pImpl->m_plannerAdvanceTimeSteps;
                           });

        if (firstMergePointAvailable != m_pImpl->mergePoints.end())
        {
            if (m_pImpl->newTrajectoryRequired)
                return true;

            m_pImpl->newTrajectoryMergeCounter = *firstMergePointAvailable;
        } else
        {
            if (m_pImpl->newTrajectoryRequired)
                return true;

            m_pImpl->newTrajectoryMergeCounter = m_pImpl->m_plannerAdvanceTimeSteps;
        }
    }

    m_pImpl->newTrajectoryRequired = true;

    return true;

    return true;
}

bool Planners::UnicycleTrajectoryGenerator::advance()
{
    constexpr auto logPrefix = "[UnicycleTrajectoryGenerator::advance]";

    if (m_pImpl->state == Impl::FSM::NotInitialized)
    {
        log()->error("{} The Unicycle planner has never been initialized.", logPrefix);
        return false;
    }

    // check if the new trajectory is required

    // if it is required, ask for the new trajectory to be computed (spawn an asynchronous thread)

    // check if it is time to merge the new trajectory

    // if yes merge the new trajectory

    // else advance the old trajectory

    m_pImpl->newTrajectoryRequired = true;

    m_pImpl->state = Impl::FSM::Running;

    return true;
}

// bool BipedalLocomotion::Planners::UnicycleTrajectoryGenerator::Impl::askForNewTrajectory()
// {
//     constexpr auto logPrefix = "[UnicycleTrajectoryGenerator::Impl::askForNewTrajectory]";

//     if (mergePoint >= m_DCMPositionDesired.size())
//     {
//         log()->error("{} The mergePoint has to be lower than the trajectory size.", logPrefix);
//         return false;
//     }

//     if (!m_trajectoryGenerator->updateTrajectories(initTime,
//                                                    m_DCMPositionDesired[mergePoint],
//                                                    m_DCMVelocityDesired[mergePoint],
//                                                    isLeftSwinging,
//                                                    measuredTransform,
//                                                    plannerDesiredInput))
//     {
//         log()->error("{} Unable to update the trajectory.", logPrefix);
//         return false;
//     }

//     return true;
// }