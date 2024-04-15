/**
 * @file UnicyclePlanner.h
 * @authors Lorenzo Moretti, Diego Ferigo, Giulio Romualdi, Stefano Dafarra
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Planners/UnicyclePlanner.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <cmath>
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

    UnicyclePlannerOutput output;

    UnicyclePlannerInput input;

    UnicyclePlannerParameters parameters;

    std::unique_ptr<::UnicycleGenerator> generator;
};

BipedalLocomotion::Planners::UnicyclePlannerInput
BipedalLocomotion::Planners::UnicyclePlannerInput::generateDummyUnicyclePlannerInput()
{
    UnicyclePlannerInput input;

    input.plannerInput = Eigen::VectorXd::Zero(3);

    iDynTree::Vector2 dcmInitialPosition, dcmInitialVelocity;
    dcmInitialPosition.zero();
    dcmInitialVelocity.zero();
    input.dcmInitialState.initialPosition = dcmInitialPosition;
    input.dcmInitialState.initialVelocity = dcmInitialVelocity;

    input.isLeftLastSwinging = false; // isLeftLastSwingingFoot (from previous planner run)

    input.initTime = 0.0;

    input.measuredTransform = iDynTree::Transform::Identity();
    input.measuredTransform.setPosition(iDynTree::Position(0.0, -0.085, 0.0));

    return input;
}

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
{
    m_pImpl = std::make_unique<UnicyclePlanner::Impl>();
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
    // template<typename T>
    // auto loadParam = [ptr, logPrefix](const std::string& paramName, T& param, const T& fallback) -> bool {
    //     if (!ptr->getParameter(paramName, param))
    //     { 
    //         log()->warn("{} Unable to find the parameter named '{}'. The default one with value [{}] will be used.", logPrefix, paramName, fallback);
    //         param = fallback;
    //     }
    //     return true;
    // };
    // clang-format on

    // initialization parameters
    // Eigen::Vector2d referencePointDistance;
    std::string unicycleControllerAsString{"direct"};

    double unicycleGain;
    double slowWhenTurningGain;
    double slowWhenBackwardFactor;
    double slowWhenSidewaysFactor;

    // double dt;
    // double plannerHorizon;

    double positionWeight;
    double timeWeight;

    std::string leftContactFrameName;
    std::string rightContactFrameName;

    double maxStepLength;
    double minStepLength;
    double maxLengthBackwardFactor;
    // double nominalWidth;
    double minWidth;
    double minStepDuration;
    double maxStepDuration;
    double nominalDuration;
    double maxAngleVariation;
    double minAngleVariation;

    Eigen::Vector2d saturationFactors;
    // double leftYawDeltaInRad;
    // double rightYawDeltaInRad;

    bool startWithLeft{true};
    bool startWithSameFoot{true};

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

    ok = ok && loadParam("referencePosition", m_pImpl->parameters.referencePointDistance);
    ok = ok && loadParam("controlType", unicycleControllerAsString);
    ok = ok && loadParam("unicycleGain", unicycleGain);
    ok = ok && loadParam("slowWhenTurningGain", slowWhenTurningGain);
    ok = ok && loadParam("slowWhenBackwardFactor", slowWhenBackwardFactor);
    ok = ok && loadParam("slowWhenSidewaysFactor", slowWhenSidewaysFactor);
    ok = ok && loadParam("dt", m_pImpl->parameters.dt);
    ok = ok && loadParam("plannerHorizon", m_pImpl->parameters.plannerHorizon);
    ok = ok && loadParam("positionWeight", positionWeight);
    ok = ok && loadParam("timeWeight", timeWeight);
    ok = ok && loadParam("maxStepLength", maxStepLength);
    ok = ok && loadParam("minStepLength", minStepLength);
    ok = ok && loadParam("maxLengthBackwardFactor", maxLengthBackwardFactor);
    ok = ok && loadParam("nominalWidth", m_pImpl->parameters.nominalWidth);
    ok = ok && loadParam("minWidth", minWidth);
    ok = ok && loadParam("minStepDuration", minStepDuration);
    ok = ok && loadParam("maxStepDuration", maxStepDuration);
    ok = ok && loadParam("nominalDuration", nominalDuration);
    ok = ok && loadParam("maxAngleVariation", maxAngleVariation);
    ok = ok && loadParam("minAngleVariation", minAngleVariation);
    ok = ok && loadParam("saturationFactors", saturationFactors);
    ok = ok && loadParam("leftYawDeltaInDeg", m_pImpl->parameters.leftYawDeltaInRad);
    ok = ok && loadParam("rightYawDeltaInDeg", m_pImpl->parameters.rightYawDeltaInRad);
    ok = ok && loadParam("swingLeft", startWithLeft);
    ok = ok && loadParam("startAlwaysSameFoot", startWithSameFoot);
    ok = ok && loadParam("mergePointRatios", mergePointRatios);
    ok = ok && loadParam("switchOverSwingRatio", switchOverSwingRatio);
    ok = ok && loadParam("lastStepSwitchTime", lastStepSwitchTime);
    ok = ok && loadParam("isPauseActive", isPauseActive);
    ok = ok && loadParam("comHeight", comHeight);
    ok = ok && loadParam("comHeightDelta", comHeightDelta);
    ok = ok && loadParam("leftZMPDelta", leftZMPDelta);
    ok = ok && loadParam("rightZMPDelta", rightZMPDelta);
    ok = ok && loadParam("lastStepDCMOffset", lastStepDCMOffset);
    ok = ok && loadParam("leftContactFrameName", leftContactFrameName);
    ok = ok && loadParam("rightContactFrameName", rightContactFrameName);

    // try to configure the planner
    m_pImpl->generator = std::make_unique<::UnicycleGenerator>();
    auto unicyclePlanner = m_pImpl->generator->unicyclePlanner();

    ok = ok
         && unicyclePlanner->setDesiredPersonDistance(m_pImpl->parameters.referencePointDistance[0],
                                                      m_pImpl->parameters.referencePointDistance[1]);
    ok = ok && unicyclePlanner->setPersonFollowingControllerGain(unicycleGain);
    ok = ok && unicyclePlanner->setSlowWhenTurnGain(slowWhenTurningGain);
    ok = ok && unicyclePlanner->setSlowWhenBackwardFactor(slowWhenBackwardFactor);
    ok = ok && unicyclePlanner->setSlowWhenSidewaysFactor(slowWhenBackwardFactor);
    ok = ok && unicyclePlanner->setMaxStepLength(maxStepLength, maxLengthBackwardFactor);
    ok = ok && unicyclePlanner->setMaximumIntegratorStepSize(m_pImpl->parameters.dt);
    ok = ok && unicyclePlanner->setWidthSetting(minWidth, m_pImpl->parameters.nominalWidth);
    ok = ok && unicyclePlanner->setMaxAngleVariation(maxAngleVariation);
    ok = ok && unicyclePlanner->setMinimumAngleForNewSteps(minAngleVariation);
    ok = ok && unicyclePlanner->setCostWeights(positionWeight, timeWeight);
    ok = ok && unicyclePlanner->setStepTimings(minStepDuration, maxStepDuration, nominalDuration);
    ok = ok && unicyclePlanner->setPlannerPeriod(m_pImpl->parameters.dt);
    ok = ok && unicyclePlanner->setMinimumStepLength(minStepLength);
    ok = ok
         && unicyclePlanner->setSaturationsConservativeFactors(saturationFactors(0),
                                                               saturationFactors(1));
    unicyclePlanner->setLeftFootYawOffsetInRadians(m_pImpl->parameters.leftYawDeltaInRad);
    unicyclePlanner->setRightFootYawOffsetInRadians(m_pImpl->parameters.rightYawDeltaInRad);
    unicyclePlanner->addTerminalStep(true);
    unicyclePlanner->startWithLeft(startWithLeft);
    unicyclePlanner->resetStartingFootIfStill(startWithSameFoot);

    auto unicycleController = getUnicycleControllerFromString(unicycleControllerAsString);
    ok = ok && unicyclePlanner->setUnicycleController(unicycleController);

    ok = ok && m_pImpl->generator->setSwitchOverSwingRatio(switchOverSwingRatio);
    ok = ok && m_pImpl->generator->setTerminalHalfSwitchTime(lastStepSwitchTime);
    ok = ok && m_pImpl->generator->setPauseConditions(maxStepDuration, nominalDuration);
    ok = ok && m_pImpl->generator->setMergePointRatio(mergePointRatios[0], mergePointRatios[1]);

    m_pImpl->generator->setPauseActive(isPauseActive);

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

    // genereateFirstTrajectory();
    ok = ok && generateFirstTrajectory();

    auto leftSteps = m_pImpl->generator->getLeftFootPrint()->getSteps();

    for (const auto& step : leftSteps)
    {
        BipedalLocomotion::log()->debug("Left step at initialization: position: {}, angle: {}, "
                                        "impact time: {}",
                                        step.position.toString(),
                                        step.angle,
                                        step.impactTime);
    }

    auto rightSteps = m_pImpl->generator->getRightFootPrint()->getSteps();

    for (const auto& step : rightSteps)
    {
        BipedalLocomotion::log()->debug("Right step at initialization: position: {}, angle: {}, "
                                        "impact time: {}",
                                        step.position.toString(),
                                        step.angle,
                                        step.impactTime);
    }

    std::vector<StepPhase> leftPhases, rightPhases;
    m_pImpl->generator->getStepPhases(leftPhases, rightPhases);

    for (size_t i = 0; i < leftPhases.size(); i++)
    {
        BipedalLocomotion::log()->debug("Left phase at initialization: {}",
                                        static_cast<int>(leftPhases.at(i)));
    }

    for (size_t i = 0; i < rightPhases.size(); i++)
    {
        BipedalLocomotion::log()->debug("Right phase at initialization: {}",
                                        static_cast<int>(rightPhases.at(i)));
    }

    if (ok)
    {
        m_pImpl->state = Impl::FSM::Initialized;
    }

    return ok;
}

const Planners::UnicyclePlannerOutput& Planners::UnicyclePlanner::getOutput() const
{
    constexpr auto logPrefix = "[UnicyclePlanner::getOutput]";

    return m_pImpl->output;
}

bool Planners::UnicyclePlanner::isOutputValid() const
{
    return m_pImpl->state == Impl::FSM::Running;
}

bool Planners::UnicyclePlanner::setInput(const UnicyclePlannerInput& input)
{
    constexpr auto logPrefix = "[UnicyclePlanner::setInput]";

    if (m_pImpl->state == Impl::FSM::NotInitialized)
    {
        log()->error("{} The Unicycle planner has never been initialized.", logPrefix);
        return false;
    }

    m_pImpl->input = input;

    return true;
}

bool Planners::UnicyclePlanner::advance()
{
    constexpr auto logPrefix = "[UnicyclePlanner::advance]";

    if (m_pImpl->state == Impl::FSM::NotInitialized)
    {
        log()->error("{} The Unicycle planner has never been initialized.", logPrefix);
        return false;
    }

    bool correctLeft{m_pImpl->input.isLeftLastSwinging};

    // set timings
    double dt{m_pImpl->parameters.dt};
    double initTime{m_pImpl->input.initTime};
    double endTime = initTime + m_pImpl->parameters.plannerHorizon;

    // set desired point
    Eigen::Vector2d desiredPointInRelativeFrame, desiredPointInAbsoluteFrame;
    desiredPointInRelativeFrame(0) = m_pImpl->input.plannerInput(0);
    desiredPointInRelativeFrame(0) = m_pImpl->input.plannerInput(1);

    // left foot
    Eigen::Vector2d measuredPositionLeft;
    double measuredAngleLeft;
    double leftYawDeltaInRad;
    measuredPositionLeft(0) = m_pImpl->input.measuredTransform.getPosition()(0);
    measuredPositionLeft(1) = m_pImpl->input.measuredTransform.getPosition()(1);
    measuredAngleLeft = m_pImpl->input.measuredTransform.getRotation().asRPY()(2);
    leftYawDeltaInRad = m_pImpl->parameters.leftYawDeltaInRad;

    // right foot
    Eigen::Vector2d measuredPositionRight;
    double measuredAngleRight;
    double rightYawDeltaInRad;
    measuredPositionRight(0) = m_pImpl->input.measuredTransform.getPosition()(0);
    measuredPositionRight(1) = m_pImpl->input.measuredTransform.getPosition()(1);
    measuredAngleRight = m_pImpl->input.measuredTransform.getRotation().asRPY()(2);
    rightYawDeltaInRad = m_pImpl->parameters.rightYawDeltaInRad;

    // get unicycle pose
    double measuredAngle;
    measuredAngle = correctLeft ? measuredAngleLeft : measuredAngleRight;
    Eigen::Vector2d measuredPosition = correctLeft ? measuredPositionLeft : measuredPositionRight;
    Eigen::Vector2d unicyclePositionFromStanceFoot, footPosition, unicyclePosition;
    unicyclePositionFromStanceFoot(0) = 0.0;

    Eigen::Matrix2d unicycleRotation;
    double unicycleAngle;

    if (correctLeft)
    {
        unicyclePositionFromStanceFoot(1) = -m_pImpl->parameters.nominalWidth / 2;
        unicycleAngle = measuredAngleLeft - leftYawDeltaInRad;
        footPosition = measuredPositionLeft;
    } else
    {
        unicyclePositionFromStanceFoot(1) = m_pImpl->parameters.nominalWidth / 2;
        unicycleAngle = measuredAngleRight - rightYawDeltaInRad;
        footPosition = measuredPositionRight;
    }

    double s_theta = std::sin(unicycleAngle);
    double c_theta = std::cos(unicycleAngle);

    unicycleRotation(0, 0) = c_theta;
    unicycleRotation(0, 1) = -s_theta;
    unicycleRotation(1, 0) = s_theta;
    unicycleRotation(1, 1) = c_theta;

    unicyclePosition = unicycleRotation * unicyclePositionFromStanceFoot + footPosition;

    // apply the homogeneous transformation w_H_{unicycle}
    desiredPointInAbsoluteFrame
        = unicycleRotation
              * (m_pImpl->parameters.referencePointDistance + desiredPointInRelativeFrame)
          + unicyclePosition;

    // clear the old trajectory
    auto unicyclePlanner = m_pImpl->generator->unicyclePlanner();
    unicyclePlanner->clearPersonFollowingDesiredTrajectory();

    // add new point
    if (!unicyclePlanner->addPersonFollowingDesiredTrajectoryPoint(endTime,
                                                                   iDynTree::Vector2(
                                                                       desiredPointInAbsoluteFrame)))
    {
        log()->error("{} Error while setting the new reference.", logPrefix);
        return false;
    }

    // set the desired direct control
    unicyclePlanner->setDesiredDirectControl(m_pImpl->input.plannerInput(0),
                                             m_pImpl->input.plannerInput(1),
                                             m_pImpl->input.plannerInput(2));

    // set the initial state of the DCM trajectory generator
    auto dcmGenerator = m_pImpl->generator->addDCMTrajectoryGenerator();

    if (!dcmGenerator->setDCMInitialState(m_pImpl->input.dcmInitialState))
    {
        log()->error("{} Failed to set the initial state.", logPrefix);
        return false;
    }

    // generate the new trajectory
    if (!(m_pImpl->generator->reGenerate(initTime,
                                         dt,
                                         endTime,
                                         correctLeft,
                                         iDynTree::Vector2(measuredPosition),
                                         measuredAngle)))
    {
        log()->error("{} Failed in computing new trajectory.", logPrefix);
        return false;
    }

    // get the feet contact status
    std::vector<bool> leftFootInContact, rightFootInContact;
    m_pImpl->generator->getFeetStandingPeriods(leftFootInContact, rightFootInContact);
    m_pImpl->output.contactStatus.leftFootInContact = leftFootInContact;
    m_pImpl->output.contactStatus.rightFootInContact = rightFootInContact;

    std::vector<bool> UsedLeftAsFixed;
    m_pImpl->generator->getWhenUseLeftAsFixed(UsedLeftAsFixed);
    m_pImpl->output.contactStatus.UsedLeftAsFixed = UsedLeftAsFixed;

    // get the contact phase lists
    BipedalLocomotion::Contacts::ContactListMap ContactListMap;
    std::vector<StepPhase> leftStepPhases, rightStepPhases;
    m_pImpl->generator->getStepPhases(leftStepPhases, rightStepPhases);

    auto leftSteps = m_pImpl->generator->getLeftFootPrint()->getSteps();
    auto rightSteps = m_pImpl->generator->getRightFootPrint()->getSteps();

    auto getContactList
        = [](const double initTime,
             const double dt,
             const std::vector<StepPhase>& stepPhases,
             const std::deque<Step>& steps,
             const int contactFrameIndex,
             const std::string& contactName) -> BipedalLocomotion::Contacts::ContactList {
        BipedalLocomotion::Contacts::ContactList contactList;

        size_t timeIndex{1};
        auto stepIterator = steps.begin();

        while (stepIterator != steps.end())
        {
            auto step = *stepIterator;

            BipedalLocomotion::Contacts::PlannedContact contact{};

            contact.index = contactFrameIndex;
            contact.name = contactName;

            Eigen::Vector3d translation = Eigen::Vector3d::Zero();
            translation.head(2) = iDynTree::toEigen(step.position);
            manif::SO3d rotation{0, 0, step.angle};
            contact.pose = manif::SE3d(translation, rotation);

            contact.activationTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
                step.impactTime * std::chrono::seconds(1));
            contact.deactivationTime = std::chrono::nanoseconds::max();

            for (auto t = timeIndex; t < stepPhases.size(); t++)
            {
                if ((stepPhases.at(t) == StepPhase::Swing)
                    && (stepPhases.at(t - 1) == StepPhase::SwitchOut))
                {
                    contact.deactivationTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        (initTime + dt * (t - 1)) * std::chrono::seconds(1));

                    timeIndex += (t + 1);
                    break;
                }
            }

            contactList.addContact(contact);
            stepIterator++;
        };

        return contactList;
    };

    auto leftContactList = getContactList(initTime,
                                          dt,
                                          leftStepPhases,
                                          leftSteps,
                                          m_pImpl->parameters.leftContactFrameIndex,
                                          "left_foot");

    auto rightContactList = getContactList(initTime,
                                           dt,
                                           rightStepPhases,
                                           rightSteps,
                                           m_pImpl->parameters.rightContactFrameIndex,
                                           "right_foot");

    ContactListMap["left_foot"] = leftContactList;
    ContactListMap["right_foot"] = rightContactList;
    m_pImpl->output.ContactPhaseList.setLists(ContactListMap);

    // get the DCM trajectory
    auto convertToEigen
        = [](const std::vector<iDynTree::Vector2>& inputVect) -> std::vector<Eigen::Vector2d> {
        std::vector<Eigen::Vector2d> outputVect;
        outputVect.reserve(inputVect.size());

        for (const auto& v : inputVect)
        {
            outputVect.push_back(iDynTree::toEigen(v));
        };

        return outputVect;
    };

    m_pImpl->output.dcmTrajectory.dcmPosition = convertToEigen(dcmGenerator->getDCMPosition());
    m_pImpl->output.dcmTrajectory.dcmVelocity = convertToEigen(dcmGenerator->getDCMVelocity());

    // get the CoM height trajectory
    auto comHeightGenerator = m_pImpl->generator->addCoMHeightTrajectoryGenerator();
    comHeightGenerator->getCoMHeightTrajectory(
        m_pImpl->output.comHeightTrajectory.comHeightPosition);
    comHeightGenerator->getCoMHeightVelocity(m_pImpl->output.comHeightTrajectory.comHeightVelocity);
    comHeightGenerator->getCoMHeightAccelerationProfile(
        m_pImpl->output.comHeightTrajectory.comHeightAcceleration);

    // get the merge points
    std::vector<size_t> mergePoints;
    m_pImpl->generator->getMergePoints(mergePoints);
    m_pImpl->output.mergePoints = mergePoints;

    m_pImpl->state = Impl::FSM::Running;

    return true;
}

bool BipedalLocomotion::Planners::UnicyclePlanner::generateFirstTrajectory()
{

    constexpr auto logPrefix = "[UnicyclePlanner::generateFirstTrajectory]";

    // clear the all trajectory
    auto unicyclePlanner = m_pImpl->generator->unicyclePlanner();
    unicyclePlanner->clearPersonFollowingDesiredTrajectory();
    unicyclePlanner->setDesiredDirectControl(0.0, 0.0, 0.0);

    // clear left and right footsteps
    m_pImpl->generator->getLeftFootPrint()->clearSteps();
    m_pImpl->generator->getRightFootPrint()->clearSteps();

    // set initial and final times
    double initTime = 0;
    double endTime = initTime + m_pImpl->parameters.plannerHorizon;

    // at the beginning iCub has to stop
    Eigen::Vector2d m_personFollowingDesiredPoint;
    m_personFollowingDesiredPoint(0) = m_pImpl->parameters.referencePointDistance(0);
    m_personFollowingDesiredPoint(1) = m_pImpl->parameters.referencePointDistance(1);

    // add the initial point
    if (!unicyclePlanner
             ->addPersonFollowingDesiredTrajectoryPoint(initTime,
                                                        iDynTree::Vector2(
                                                            m_personFollowingDesiredPoint)))
    {
        log()->error("{} Error while setting the initial point.", logPrefix);
        return false;
    }

    // add the final point
    if (!unicyclePlanner
             ->addPersonFollowingDesiredTrajectoryPoint(endTime,
                                                        iDynTree::Vector2(
                                                            m_personFollowingDesiredPoint)))
    {
        log()->error("{} Error while setting the final point.", logPrefix);
        return false;
    }

    // generate the first trajectories
    if (!m_pImpl->generator->generate(initTime, m_pImpl->parameters.dt, endTime))
    {

        log()->error("{} Error while computing the first trajectories.", logPrefix);

        return false;
    }

    return true;
}
