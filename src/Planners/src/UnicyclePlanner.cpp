/**
 * @file UnicyclePlanner.h
 * @authors Lorenzo Moretti, Diego Ferigo, Giulio Romualdi, Stefano Dafarra
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include "BipedalLocomotion/Math/Constants.h"
#include <BipedalLocomotion/Planners/UnicyclePlanner.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <FootPrint.h>
#include <UnicycleGenerator.h>
#include <UnicyclePlanner.h>
#include <chrono>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/VectorFixSize.h>
#include <manif/SE3.h>

#include <cassert>
#include <limits>
#include <string>

using namespace BipedalLocomotion;

class Planners::UnicyclePlanner::Impl
{

public:
    enum class FSM
    {
        NotInitialized,
        Initialized,
        Running,
    };

    FSM state{FSM::NotInitialized};

    struct
    {
        std::optional<Contacts::PlannedContact> left;
        std::optional<Contacts::PlannedContact> right;
    } initialContacts;

    UnicyclePlannerOutput outputRef;
    std::optional<UnicyclePlannerOutput> output = std::nullopt;

    std::unique_ptr<::UnicycleGenerator> generator;
};

UnicycleController Planners::UnicyclePlanner::getUnicycleControllerFromString(
    const std::string& unicycleControllerAsString)
{
    if (unicycleControllerAsString == "personFollowing")
    {
        return UnicycleController::PERSON_FOLLOWING;
    } else if (unicycleControllerAsString == "direct")
    {
        return UnicycleController::DIRECT;
    } else
    {
        return UnicycleController::DIRECT;
    }
}

Planners::UnicyclePlanner::UnicyclePlanner()
    : m_pImpl{std::make_unique<Impl>()}
{
}

Planners::UnicyclePlanner::~UnicyclePlanner() = default;

bool Planners::UnicyclePlanner::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[UnicyclePlanner::initialize]";

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

    // TO DO: write a variant of the lambda function with fallback option
    // clang-format off
    // auto loadParam = [ptr, logPrefix](const std::string& paramName, auto& param, auto fallback) -> bool {
    //     if (!ptr->getParameter(paramName, param))
    //     { 
    //         log()->warn("{} Unable to find the parameter named '{}'. The default one with value [{}] will be used.", logPrefix, paramName, fallback);
    //     }
    //     return true;
    // };
    // clang-format on

    // initialization parameters
    Eigen::Vector2d referencePointDistance;
    std::string unicycleControllerAsString{"direct"};

    double unicycleGain;
    double slowWhenTurningGain;
    double slowWhenBackwardFactor;
    double slowWhenSidewaysFactor;

    double dt;

    double positionWeight;
    double timeWeight;

    double maxStepLength;
    double minStepLength;
    double maxLengthBackwardFactor;
    double nominalWidth;
    double minWidth;
    double minStepDuration;
    double maxStepDuration;
    double nominalDuration;
    double maxAngleVariation;
    double minAngleVariation;

    Eigen::Vector2d saturationFactors;
    double leftYawDeltaInRad;
    double rightYawDeltaInRad;

    bool startWithLeft{true};
    bool startWithSameFoot{true};

    double freeSpaceConservativeFactor{2.0};
    double innerEllipseSemiMajorOffset{0.0};
    double innerEllipseSemiMinorOffset{0.0};

    Eigen::Vector2d mergePointRatios;
    double switchOverSwingRatio;
    double lastStepSwitchTime;
    bool isPauseActive{true};

    double comHeight;
    double comHeightDelta;
    Eigen::Vector2d leftZMPDelta;
    Eigen::Vector2d rightZMPDelta;
    double lastStepDCMOffset;

    // parse initialization parameters
    bool ok = true;

    ok = ok && loadParam("referencePosition", referencePointDistance);
    ok = ok && loadParam("controlType", unicycleControllerAsString);
    ok = ok && loadParam("unicycleGain", unicycleGain);
    ok = ok && loadParam("slowWhenTurningGain", slowWhenTurningGain);
    ok = ok && loadParam("slowWhenBackwardFactor", slowWhenBackwardFactor);
    ok = ok && loadParam("slowWhenSidewaysFactor", slowWhenSidewaysFactor);
    ok = ok && loadParam("dt", dt);
    ok = ok && loadParam("positionWeight", positionWeight);
    ok = ok && loadParam("timeWeight", timeWeight);
    ok = ok && loadParam("maxStepLength", maxStepLength);
    ok = ok && loadParam("minStepLength", minStepLength);
    ok = ok && loadParam("maxLengthBackwardFactor", maxLengthBackwardFactor);
    ok = ok && loadParam("nominalWidth", nominalWidth);
    ok = ok && loadParam("minWidth", minWidth);
    ok = ok && loadParam("minStepDuration", minStepDuration);
    ok = ok && loadParam("maxStepDuration", maxStepDuration);
    ok = ok && loadParam("nominalDuration", nominalDuration);
    ok = ok && loadParam("maxAngleVariation", maxAngleVariation);
    ok = ok && loadParam("minAngleVariation", minAngleVariation);
    ok = ok && loadParam("saturationFactors", saturationFactors);
    ok = ok && loadParam("leftYawDeltaInDeg", leftYawDeltaInRad);
    ok = ok && loadParam("rightYawDeltaInDeg", rightYawDeltaInRad);
    ok = ok && loadParam("swingLeft", startWithLeft);
    ok = ok && loadParam("startAlwaysSameFoot", startWithSameFoot);
    ok = ok && loadParam("conservative_factor", freeSpaceConservativeFactor);
    ok = ok && loadParam("inner_offset_major", innerEllipseSemiMajorOffset);
    ok = ok && loadParam("inner_offset_minor", innerEllipseSemiMinorOffset);
    ok = ok && loadParam("mergePointRatios", mergePointRatios);
    ok = ok && loadParam("switchOverSwingRatio", switchOverSwingRatio);
    ok = ok && loadParam("lastStepSwitchTime", lastStepSwitchTime);
    ok = ok && loadParam("isPauseActive", isPauseActive);
    ok = ok && loadParam("comHeight", comHeight);
    ok = ok && loadParam("comHeightDelta", comHeightDelta);
    ok = ok && loadParam("leftZMPDelta", leftZMPDelta);
    ok = ok && loadParam("rightZMPDelta", rightZMPDelta);
    ok = ok && loadParam("lastStepDCMOffset", lastStepDCMOffset);

    // try to configure the planner
    m_pImpl->generator = std::make_unique<::UnicycleGenerator>();
    auto unicyclePlanner = m_pImpl->generator->unicyclePlanner();

    ok = ok
         && unicyclePlanner->setDesiredPersonDistance(referencePointDistance[0],
                                                      referencePointDistance[1]);
    ok = ok && unicyclePlanner->setPersonFollowingControllerGain(unicycleGain);
    ok = ok && unicyclePlanner->setSlowWhenTurnGain(slowWhenTurningGain);
    ok = ok && unicyclePlanner->setSlowWhenBackwardFactor(slowWhenBackwardFactor);
    ok = ok && unicyclePlanner->setSlowWhenSidewaysFactor(slowWhenBackwardFactor);
    ok = ok && unicyclePlanner->setMaxStepLength(maxStepLength, maxLengthBackwardFactor);
    ok = ok && unicyclePlanner->setMaximumIntegratorStepSize(dt);
    ok = ok && unicyclePlanner->setWidthSetting(minWidth, nominalWidth);
    ok = ok && unicyclePlanner->setMaxAngleVariation(maxAngleVariation);
    ok = ok && unicyclePlanner->setMinimumAngleForNewSteps(minAngleVariation);
    ok = ok && unicyclePlanner->setCostWeights(positionWeight, timeWeight);
    ok = ok && unicyclePlanner->setStepTimings(minStepDuration, maxStepDuration, nominalDuration);
    ok = ok && unicyclePlanner->setPlannerPeriod(dt);
    ok = ok && unicyclePlanner->setMinimumStepLength(minStepLength);
    ok = ok
         && unicyclePlanner->setSaturationsConservativeFactors(saturationFactors(0),
                                                               saturationFactors(1));
    unicyclePlanner->setLeftFootYawOffsetInRadians(leftYawDeltaInRad);
    unicyclePlanner->setRightFootYawOffsetInRadians(rightYawDeltaInRad);
    unicyclePlanner->addTerminalStep(true);
    unicyclePlanner->startWithLeft(startWithLeft);
    unicyclePlanner->resetStartingFootIfStill(startWithSameFoot);
    ok = ok && unicyclePlanner->setFreeSpaceEllipseConservativeFactor(freeSpaceConservativeFactor);
    ok = ok
         && unicyclePlanner->setInnerFreeSpaceEllipseOffsets(innerEllipseSemiMajorOffset,
                                                             innerEllipseSemiMinorOffset);
    auto unicycleController = getUnicycleControllerFromString(unicycleControllerAsString);
    ok = ok && unicyclePlanner->setUnicycleController(unicycleController);

    ok = ok && m_pImpl->generator->setSwitchOverSwingRatio(switchOverSwingRatio);
    ok = ok && m_pImpl->generator->setTerminalHalfSwitchTime(lastStepSwitchTime);
    ok = ok && m_pImpl->generator->setPauseConditions(maxStepDuration, nominalDuration);
    ok = ok && m_pImpl->generator->setMergePointRatio(mergePointRatios[0], mergePointRatios[1]);

    m_pImpl->generator->setPauseActive(isPauseActive);

    auto comHeightGenerator = m_pImpl->generator->addCoMHeightTrajectoryGenerator();
    ok = ok && comHeightGenerator->setCoMHeightSettings(comHeight, comHeightDelta);

    auto dcmGenerator = m_pImpl->generator->addDCMTrajectoryGenerator();
    iDynTree::Vector2 leftZMPDeltaVec{leftZMPDelta};
    iDynTree::Vector2 rightZMPDeltaVec{rightZMPDelta};
    dcmGenerator->setFootOriginOffset(leftZMPDeltaVec, rightZMPDeltaVec);
    dcmGenerator->setOmega(
        sqrt(BipedalLocomotion::Math::StandardAccelerationOfGravitation / comHeight));
    dcmGenerator->setFirstDCMTrajectoryMode(FirstDCMTrajectoryMode::FifthOrderPoly);
    ok = ok && dcmGenerator->setLastStepDCMOffsetPercentage(lastStepDCMOffset);

    // m_correctLeft = true;

    // m_newFreeSpaceEllipse = false;

    if (ok)
    {
        m_pImpl->state = Impl::FSM::Initialized;
    }

    return ok;
}

const Planners::UnicyclePlannerOutput& Planners::UnicyclePlanner::getOutput() const
{
    constexpr auto logPrefix = "[UnicyclePlanner::getOutput]";

    if (!this->isOutputValid())
    {
        log()->warn("{} Returning an empty output.", logPrefix);
        m_pImpl->outputRef = {};
        return m_pImpl->outputRef;
    }

    m_pImpl->outputRef = *m_pImpl->output;
    return m_pImpl->outputRef;
}

bool Planners::UnicyclePlanner::isOutputValid() const
{
    constexpr auto logPrefix = "[UnicyclePlanner::isOutputValid]";

    if (!m_pImpl->isInitialized)
    {
        log()->error("{} The Unicycle planner has never been initialized.", logPrefix);
        return false;
    }

    if (!(m_pImpl->generator->getLeftFootPrint()->numberOfSteps() > 0)
        || !(m_pImpl->generator->getRightFootPrint()->numberOfSteps() > 0))
    {
        log()->error("{} The Unicycle planner never computed the foot steps.", logPrefix);
        return false;
    }

    if (!m_pImpl->output)
    {
        log()->error("{} The output has never been computed.", logPrefix);
        return false;
    }

    return true;
}

bool Planners::UnicyclePlanner::setInput(const UnicyclePlannerInput& input)
{
    constexpr auto logPrefix = "[UnicyclePlanner::setInput]";

    if (!m_pImpl->isInitialized)
    {
        log()->error("{} The Unicycle planner has never been initialized.", logPrefix);
        return false;
    }

    auto getMaxKnotTime = [](const UnicyclePlannerInput& input) -> double {
        double maxKnotTime = 0.0;

        for (const auto& knot : input.knots)
        {
            if (knot.time > maxKnotTime)
                maxKnotTime = knot.time;
        }

        return maxKnotTime;
    };

    if (input.tf < getMaxKnotTime(input))
    {
        log()->error("{} The input contains a knot whose time is over the planner horizon.",
                     logPrefix);
        return false;
    }

    m_pImpl->generator->unicyclePlanner()->clearPersonFollowingDesiredTrajectory();

    for (const auto& knot : input.knots)
    {
        auto position = iDynTree::Vector2();
        position[0] = knot.x;
        position[1] = knot.y;

        auto velocity = iDynTree::Vector2();
        velocity[0] = knot.dx;
        velocity[1] = knot.dy;

        if (!m_pImpl->generator->unicyclePlanner()
                 ->addPersonFollowingDesiredTrajectoryPoint(knot.time, position, velocity))
        {
            m_pImpl->generator->unicyclePlanner()->clearPersonFollowingDesiredTrajectory();
            log()->error("{} Failed to insert knot in the Unicycle planner.", logPrefix);
            return false;
        }
    }

    m_pImpl->horizon.t0 = input.t0;
    m_pImpl->horizon.tf = input.tf;

    m_pImpl->initialContacts.left = input.initialLeftContact;
    m_pImpl->initialContacts.right = input.initialRightContact;

    return true;
}

bool Planners::UnicyclePlanner::advance()
{
    constexpr auto logPrefix = "[UnicyclePlanner::advance]";

    if (!m_pImpl->isInitialized)
    {
        log()->error("{} The Unicycle planner has never been initialized.", logPrefix);
        return false;
    }

    // Lambda to clean up resources when returning false
    auto cleanup = [&]() {
        m_pImpl->generator->getLeftFootPrint()->clearSteps();
        m_pImpl->generator->getRightFootPrint()->clearSteps();
        m_pImpl->output = std::nullopt;
    };

    // Cleanup first
    cleanup();

    // ==================================
    // Plan contacts with UnicyclePlanner
    // ==================================

    // Convert manif to iDynTree
    auto toiDynTree = [](const manif::SE3d::Translation& translation) -> iDynTree::Vector2 {
        iDynTree::Vector2 position;
        position[0] = translation[0];
        position[1] = translation[1];
        return position;
    };

    // The initTime of the UnicyclePlanner cannot be smaller than
    // the impact time of the last step.
    // If an initial step configuration is passed, the initial time must be updated.
    double initTime = m_pImpl->horizon.t0;

    // Process the initial left contact configuration
    if (m_pImpl->initialContacts.left)
    {
        const auto& contact = m_pImpl->initialContacts.left;

        // Here we decompose the quaternion to YXZ intrinsic Euler angles (default in Eigen).
        // The most reliable decomposition is ZXY extrinsic, that is equivalent since the two
        // commute when the order is reversed. This decomposition, having Z as first rotation,
        // should get the correct yaw in most cases.
        const auto& euler
            = contact->pose.quat().normalized().toRotationMatrix().eulerAngles(1, 0, 2);

        // Create the inital step
        m_pImpl->generator->getLeftFootPrint()
            ->addStep(toiDynTree(contact->pose.translation()),
                      euler[2],
                      std::chrono::duration<double>(contact->activationTime).count());

        const double impactTime = std::chrono::duration<double>(contact->activationTime).count();
        initTime = impactTime > initTime ? impactTime : initTime;
    }

    // Process the initial right contact configuration
    if (m_pImpl->initialContacts.right)
    {
        const auto& contact = m_pImpl->initialContacts.right;

        // Here we decompose the quaternion to YXZ intrinsic Euler angles (default in Eigen).
        // The most reliable decomposition is ZXY extrinsic, that is equivalent since the two
        // commute when the order is reversed. This decomposition, having Z as first rotation,
        // should get the correct yaw in most cases.
        const auto& euler
            = contact->pose.quat().normalized().toRotationMatrix().eulerAngles(1, 0, 2);

        // Create the inital step
        m_pImpl->generator->getRightFootPrint()
            ->addStep(toiDynTree(contact->pose.translation()),
                      euler[2],
                      std::chrono::duration<double>(contact->activationTime).count());

        const double impactTime = std::chrono::duration<double>(contact->activationTime).count();
        initTime = impactTime > initTime ? impactTime : initTime;
    }

    if (!m_pImpl->generator->unicyclePlanner()
             ->computeNewSteps(m_pImpl->generator->getLeftFootPrint(),
                               m_pImpl->generator->getRightFootPrint(),
                               initTime,
                               m_pImpl->horizon.tf))
    {
        cleanup();
        log()->error("{} Failed to compute new steps.", logPrefix);
        return false;
    }

    // ===========================================
    // Compute step timings with UnicycleGenerator
    // ===========================================

    // Create and configure the generator
    m_pImpl->generator->setSwitchOverSwingRatio(m_pImpl->gait.stancePhaseRatio);
    m_pImpl->generator->setPauseConditions(m_pImpl->duration.max, m_pImpl->duration.nominal);

    // The last step will have an infinite deactivation time, the following option
    // is necessary for the generator but it does not affect the advanceable output
    m_pImpl->generator->setTerminalHalfSwitchTime(1.0);

    // Due to how the generator works, the start time must be bigger than last impact time
    const double startLeft = m_pImpl->generator->getLeftFootPrint()->getSteps().front().impactTime;
    const double startRight
        = m_pImpl->generator->getRightFootPrint()->getSteps().front().impactTime;
    const double startTime = std::max(startLeft, startRight);

    // Compute the contact states using the generator
    if (!m_pImpl->generator->generateFromFootPrints(m_pImpl->generator->getLeftFootPrint(),
                                                    m_pImpl->generator->getRightFootPrint(),
                                                    startTime,
                                                    m_pImpl->dt.planner))
    {
        cleanup();
        log()->error("{} Failed to generate from footprints.", logPrefix);
        return false;
    }

    // Get the contact states over the horizon
    std::vector<bool> leftStandingPeriod;
    std::vector<bool> rightStandingPeriod;
    m_pImpl->generator->getFeetStandingPeriods(leftStandingPeriod, rightStandingPeriod);

    // Bind dt to catch it in the next lambda
    const auto& dt = m_pImpl->dt.planner;

    // Lambda to convert Step to Contact, filling only the timings.
    // Transforms will be included in a later stage.
    auto convertStepsToContacts
        = [dt](const std::vector<bool>& isFootInContactVector,
               const StepList& steps) -> std::vector<Contacts::PlannedContact> {
        std::vector<Contacts::PlannedContact> contacts;

        for (const auto& step : steps)
        {
            using namespace std::chrono_literals;
            auto contact = Contacts::PlannedContact();
            contact.name = step.footName;
            contact.activationTime
                = std::chrono::duration_cast<std::chrono::nanoseconds>(step.impactTime * 1s);
            contact.deactivationTime = contact.activationTime;
            contacts.push_back(contact);
        }

        size_t contactIdx = 0;
        double activeTime = 0.0;

        for (size_t idx = 1; idx < isFootInContactVector.size(); ++idx)
        {
            // Get the active contact
            auto& contact = contacts[contactIdx];

            const bool thisState = isFootInContactVector[idx];
            const bool lastState = isFootInContactVector[idx - 1];

            // Increase the active time
            activeTime += dt;

            // During impact, reset the active time counter
            if (lastState == 0 && thisState == 1)
                activeTime = 0.0;

            // During lift, store the time in the active contact and get the new contact
            if (lastState == 1 && thisState == 0)
            {
                using namespace std::chrono_literals;
                contact.deactivationTime
                    = contact.activationTime
                      + std::chrono::duration_cast<std::chrono::nanoseconds>(activeTime * 1s);
                contactIdx++;
            }
        }

        // The deactivation time of the last contact has not yet been processed
        contacts.back().deactivationTime = std::chrono::nanoseconds::max();

        return contacts;
    };

    // Convert Step objects to PlannedContact objects
    std::vector<Contacts::PlannedContact> leftContacts
        = convertStepsToContacts(leftStandingPeriod,
                                 m_pImpl->generator->getLeftFootPrint()->getSteps());
    std::vector<Contacts::PlannedContact> rightContacts
        = convertStepsToContacts(rightStandingPeriod,
                                 m_pImpl->generator->getRightFootPrint()->getSteps());

    if (m_pImpl->generator->getLeftFootPrint()->getSteps().size() != leftContacts.size())
    {
        cleanup();
        log()->error("{} Wrong number of converted steps for left foot.", logPrefix);
        return false;
    }

    if (m_pImpl->generator->getRightFootPrint()->getSteps().size() != rightContacts.size())
    {
        cleanup();
        log()->error("{} Wrong number of converted steps for right foot.", logPrefix);
        return false;
    }

    // =======================================
    // Convert Step transforms to Contact pose
    // =======================================

    // Lambda to fill the transforms of the PlannedContact objects
    auto fillContactTransform = [](std::vector<Contacts::PlannedContact>& contacts,
                                   decltype(::FootPrint().getSteps())& steps) -> void {
        assert(contacts.size() == steps.size());
        for (size_t i = 0; i < contacts.size(); ++i)
        {
            contacts[i].pose.quat(Eigen::AngleAxisd(steps[i].angle, Eigen::Vector3d::UnitZ()));
            contacts[i].pose.translation({steps[i].position[0], steps[i].position[1], 0.0});
        }
    };

    // Fill the transforms
    fillContactTransform(leftContacts, m_pImpl->generator->getLeftFootPrint()->getSteps());
    fillContactTransform(rightContacts, m_pImpl->generator->getRightFootPrint()->getSteps());

    // ================================
    // Create the output data structure
    // ================================

    // Lambda to convert vector of PlannedContact to ConctactList
    auto convertToContactList
        = [](const std::vector<Contacts::PlannedContact>& contacts) -> Contacts::ContactList {
        Contacts::ContactList list;
        for (const auto& contact : contacts)
            list.addContact(contact);
        return list;
    };

    // Create the system's output
    m_pImpl->output = std::make_optional<UnicyclePlannerOutput>();

    m_pImpl->output->left = convertToContactList(leftContacts);
    m_pImpl->output->right = convertToContactList(rightContacts);

    m_pImpl->output->left.setDefaultName(m_pImpl->names.left);
    m_pImpl->output->right.setDefaultName(m_pImpl->names.right);

    return true;
}
