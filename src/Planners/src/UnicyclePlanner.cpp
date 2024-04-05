/**
 * @file UnicyclePlanner.h
 * @authors Diego Ferigo, Stefano Dafarra
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Planners/UnicyclePlanner.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <FootPrint.h>
#include <UnicycleGenerator.h>
#include <UnicyclePlanner.h>
#include <chrono>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/VectorFixSize.h>
#include <manif/SE3.h>

#include <cassert>
#include <limits>

using namespace BipedalLocomotion;

class Planners::UnicyclePlanner::Impl
{
public:
    struct
    {
        double planner;
    } dt;

    struct
    {
        double t0;
        double tf;
    } horizon;

    struct
    {
        struct
        {
            double x = 0.10;
            double y = 0.00;
        } reference;

        struct
        {
            double unicycle = 10.0;
            double slowWhenTurning = 0.0;
        } gains;
    } controller;

    struct
    {
        double time = 1.0;
        double position = 1.0;
    } weights;

    struct
    {
        double min;
        double max;
        double nominal;
    } duration;

    struct
    {
        double min;
        double max;
    } stepLength;

    struct
    {
        double min;
        double nominal;
    } feetDistance;

    struct
    {
        double min;
        double max;
    } angleVariation;

    struct
    {
        double stancePhaseRatio;
        bool startWithLeft = false;
        bool terminalStep = true;
        bool resetStartingFootIfStill = false;
    } gait;

    struct
    {
        std::string left = "left";
        std::string right = "right";
    } names;

    struct
    {
        std::optional<Contacts::PlannedContact> left;
        std::optional<Contacts::PlannedContact> right;
    } initialContacts;

    UnicyclePlannerOutput outputRef;
    std::optional<UnicyclePlannerOutput> output = std::nullopt;

    std::shared_ptr<::FootPrint> left;
    std::shared_ptr<::FootPrint> right;
    std::unique_ptr<::UnicyclePlanner> planner;
};

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
        log()->error("{} The handler has to point to an already initialized IParametersHandler.",
                     logPrefix);
        return false;
    }

    bool okPlanner = true;
    m_pImpl->planner = std::make_unique<::UnicyclePlanner>();

    // ==
    // dt
    // ==

    if (!ptr->getParameter("sampling_time", m_pImpl->dt.planner))
    {
        log()->error("{} Unable to load the sampling time of the planner (sampling_time).",
                     logPrefix);
        return false;
    }

    okPlanner = okPlanner && m_pImpl->planner->setPlannerPeriod(m_pImpl->dt.planner);
    okPlanner = okPlanner && m_pImpl->planner->setMaximumIntegratorStepSize(m_pImpl->dt.planner);

    // ==========
    // controller
    // ==========

    if (!ptr->getParameter("unicycleGain", m_pImpl->controller.gains.unicycle))
    {
        log()->info("{} Using default unicycleGain={}.",
                    logPrefix,
                    m_pImpl->controller.gains.unicycle);
    }

    if (!ptr->getParameter("slowWhenTurningGain", m_pImpl->controller.gains.slowWhenTurning))
    {
        log()->info("{} Using default slowWhenTurningGain={}.",
                    logPrefix,
                    m_pImpl->controller.gains.slowWhenTurning);
    }

    okPlanner = okPlanner
                && m_pImpl->planner->setControllerGain(m_pImpl->controller.gains.unicycle);
    okPlanner = okPlanner
                && m_pImpl->planner->setSlowWhenTurnGain(m_pImpl->controller.gains.slowWhenTurning);

    std::vector<double> reference;

    if (!(ptr->getParameter("referencePosition", reference) && reference.size() == 2))
    {
        log()->info("{} Using default referencePosition=({}, {}).",
                    logPrefix,
                    m_pImpl->controller.reference.x,
                    m_pImpl->controller.reference.y);
    } else
    {
        m_pImpl->controller.reference.x = reference[0];
        m_pImpl->controller.reference.y = reference[1];
    }

    okPlanner = okPlanner
                && m_pImpl->planner->setDesiredPersonDistance(m_pImpl->controller.reference.x,
                                                              m_pImpl->controller.reference.y);

    // =======
    // weights
    // =======

    if (!ptr->getParameter("timeWeight", m_pImpl->weights.time))
    {
        log()->info("{} Using default timeWeight={}.", logPrefix, m_pImpl->weights.time);
    }

    if (!ptr->getParameter("positionWeight", m_pImpl->weights.position))
    {
        log()->info("{} Using default positionWeight={}.", logPrefix, m_pImpl->weights.position);
    }

    okPlanner
        = okPlanner
          && m_pImpl->planner->setCostWeights(m_pImpl->weights.position, m_pImpl->weights.time);

    // ========
    // duration
    // ========

    if (!ptr->getParameter("minStepDuration", m_pImpl->duration.min))
    {
        log()->error("{} Unable to load the min step duration (minStepDuration).", logPrefix);
        return false;
    }

    if (!ptr->getParameter("maxStepDuration", m_pImpl->duration.max))
    {
        log()->error("{} Unable to load the max step duration (maxStepDuration).", logPrefix);
        return false;
    }

    if (!ptr->getParameter("nominalDuration", m_pImpl->duration.nominal))
    {
        log()->error("{} Unable to load the nominal step duration (nominalDuration).", logPrefix);
        return false;
    }

    okPlanner = okPlanner
                && m_pImpl->planner->setStepTimings(m_pImpl->duration.min,
                                                    m_pImpl->duration.max,
                                                    m_pImpl->duration.nominal);

    // ==========
    // stepLength
    // ==========

    if (!ptr->getParameter("minStepLength", m_pImpl->stepLength.min))
    {
        log()->error("{} Unable to load the min step length (minStepLength).", logPrefix);
        return false;
    }

    if (!ptr->getParameter("maxStepLength", m_pImpl->stepLength.max))
    {
        log()->error("{} Unable to load the max step length (maxStepLength).", logPrefix);
        return false;
    }

    okPlanner = okPlanner && m_pImpl->planner->setMaxStepLength(m_pImpl->stepLength.max);
    okPlanner = okPlanner && m_pImpl->planner->setMinimumStepLength(m_pImpl->stepLength.min);

    // ============
    // feetDistance
    // ============

    if (!ptr->getParameter("minWidth", m_pImpl->feetDistance.min))
    {
        log()->error("{} Unable to load the min feet distance (minWidth).", logPrefix);
        return false;
    }

    if (!ptr->getParameter("nominalWidth", m_pImpl->feetDistance.nominal))
    {
        log()->error("{} Unable to load the nominal feet distance (nominalWidth).", logPrefix);
        return false;
    }

    okPlanner = okPlanner
                && m_pImpl->planner->setWidthSetting( //
                    m_pImpl->feetDistance.min,
                    m_pImpl->feetDistance.nominal);

    // ==============
    // angleVariation
    // ==============

    if (!ptr->getParameter("minAngleVariation", m_pImpl->angleVariation.min))
    {
        log()->error("{} Unable to load the min foot angle variation (minAngleVariation).",
                     logPrefix);
        return false;
    }

    if (!ptr->getParameter("maxAngleVariation", m_pImpl->angleVariation.max))
    {
        log()->error("{} Unable to load the max foot angle variation (maxAngleVariation).",
                     logPrefix);
        return false;
    }

    okPlanner = okPlanner && m_pImpl->planner->setMaxAngleVariation(m_pImpl->angleVariation.max);
    okPlanner = okPlanner
                && m_pImpl->planner->setMinimumAngleForNewSteps(m_pImpl->angleVariation.min);

    // ====
    // gait
    // ====

    if (!ptr->getParameter("switchOverSwingRatio", m_pImpl->gait.stancePhaseRatio))
    {
        log()->error("{} Unable to load the stance phase ratio (switchOverSwingRatio).", logPrefix);
        return false;
    }

    if (m_pImpl->gait.stancePhaseRatio <= 0)
    {
        log()->error("{} The switchOverSwingRatio cannot be <= 0.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("swingLeft", m_pImpl->gait.startWithLeft))
    {
        log()->info("{} Using default swingLeft={}.", logPrefix, m_pImpl->gait.startWithLeft);
    }

    if (!ptr->getParameter("terminalStep", m_pImpl->gait.terminalStep))
    {
        log()->info("{} Using default terminalStep={}.", logPrefix, m_pImpl->gait.terminalStep);
    }

    if (!ptr->getParameter("startAlwaysSameFoot", m_pImpl->gait.resetStartingFootIfStill))
    {
        log()->info("{} Using default startAlwaysSameFoot={}.",
                    logPrefix,
                    m_pImpl->gait.resetStartingFootIfStill);
    }

    m_pImpl->planner->startWithLeft(m_pImpl->gait.startWithLeft);
    m_pImpl->planner->addTerminalStep(m_pImpl->gait.terminalStep);
    m_pImpl->planner->resetStartingFootIfStill(m_pImpl->gait.resetStartingFootIfStill);

    // =====
    // names
    // =====

    if (!ptr->getParameter("left_foot_name", m_pImpl->names.left))
    {
        log()->info("{} Using default left_foot_name={}.", logPrefix, m_pImpl->names.left);
    }

    if (!ptr->getParameter("right_foot_name", m_pImpl->names.right))
    {
        log()->info("{} Using default right_foot_name={}.", logPrefix, m_pImpl->names.right);
    }

    // =============================
    // UnicyclePlanner configuration
    // =============================

    if (!okPlanner)
    {
        log()->error("{} Failed to configure UnicyclePlanner.", logPrefix);
        return false;
    }

    return true;
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

    if (!m_pImpl->planner)
    {
        log()->error("{} The Unicycle planner has never been initialized.", logPrefix);
        return false;
    }

    if (!m_pImpl->left || !m_pImpl->right)
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

    if (!m_pImpl->planner)
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

    m_pImpl->planner->clearDesiredTrajectory();

    for (const auto& knot : input.knots)
    {
        auto position = iDynTree::Vector2();
        position[0] = knot.x;
        position[1] = knot.y;

        auto velocity = iDynTree::Vector2();
        velocity[0] = knot.dx;
        velocity[1] = knot.dy;

        if (!m_pImpl->planner->addDesiredTrajectoryPoint(knot.time, position, velocity))
        {
            m_pImpl->planner->clearDesiredTrajectory();
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

    if (!m_pImpl->planner)
    {
        log()->error("{} The Unicycle planner has never been initialized.", logPrefix);
        return false;
    }

    // Lambda to clean up resources when returning false
    auto cleanup = [&]() {
        m_pImpl->left = nullptr;
        m_pImpl->right = nullptr;
        m_pImpl->output = std::nullopt;
    };

    // Cleanup first
    cleanup();

    // ==================================
    // Plan contacts with UnicyclePlanner
    // ==================================

    // Initialize the left FootPrint
    m_pImpl->left = std::make_shared<FootPrint>();
    m_pImpl->left->setFootName(m_pImpl->names.left);

    // Initialize the right FootPrint
    m_pImpl->right = std::make_shared<FootPrint>();
    m_pImpl->right->setFootName(m_pImpl->names.right);

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
        m_pImpl->left->addStep(toiDynTree(contact->pose.translation()),
                               euler[2],
                               std::chrono::duration<double>(contact->activationTime).count());

        const double impactTime = std::chrono::duration<double>(contact->activationTime).count();
        initTime = impactTime > initTime ? impactTime : initTime;
    }

    // Process the initial left contact configuration
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
        m_pImpl->right->addStep(toiDynTree(contact->pose.translation()),
                                euler[2],
                                std::chrono::duration<double>(contact->activationTime).count());

        const double impactTime = std::chrono::duration<double>(contact->activationTime).count();
        initTime = impactTime > initTime ? impactTime : initTime;
    }

    if (!m_pImpl->planner->computeNewSteps(m_pImpl->left,
                                           m_pImpl->right,
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
    auto generator = UnicycleGenerator();
    generator.setSwitchOverSwingRatio(m_pImpl->gait.stancePhaseRatio);
    generator.setPauseConditions(m_pImpl->duration.max, m_pImpl->duration.nominal);

    // The last step will have an infinite deactivation time, the following option
    // is necessary for the generator but it does not affect the advanceable output
    generator.setTerminalHalfSwitchTime(1.0);

    // Due to how the generator works, the start time must be bigger than last impact time
    const double startLeft = m_pImpl->left->getSteps().front().impactTime;
    const double startRight = m_pImpl->right->getSteps().front().impactTime;
    const double startTime = std::max(startLeft, startRight);

    // Compute the contact states using the generator
    if (!generator.generateFromFootPrints(m_pImpl->left,
                                          m_pImpl->right,
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
    generator.getFeetStandingPeriods(leftStandingPeriod, rightStandingPeriod);

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
        = convertStepsToContacts(leftStandingPeriod, m_pImpl->left->getSteps());
    std::vector<Contacts::PlannedContact> rightContacts
        = convertStepsToContacts(rightStandingPeriod, m_pImpl->right->getSteps());

    if (m_pImpl->left->getSteps().size() != leftContacts.size())
    {
        cleanup();
        log()->error("{} Wrong number of converted steps for left foot.", logPrefix);
        return false;
    }

    if (m_pImpl->right->getSteps().size() != rightContacts.size())
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
    fillContactTransform(leftContacts, m_pImpl->left->getSteps());
    fillContactTransform(rightContacts, m_pImpl->right->getSteps());

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
