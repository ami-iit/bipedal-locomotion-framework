/**
 * @file SchmittTriggerDetector.cpp
 * @authors Prashanth Ramadoss, Giulio Romualdi
 * @copyright 2020-2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContactDetectors/SchmittTriggerDetector.h>
#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Math/SchmittTrigger.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <chrono>

namespace blf = BipedalLocomotion;
using namespace BipedalLocomotion::Contacts;
using namespace BipedalLocomotion::ParametersHandler;

struct SchmittTriggerDetector::Impl
{
    /** Container for SchmittTrigger for each contact */
    std::unordered_map<std::string, blf::Math::SchmittTrigger> manager;

    /**
     * Utility function to check if contact exists
     */
    [[nodiscard]] inline bool contactExists(const std::string& contactName) const
    {
        return manager.find(contactName) != manager.end();
    }
};

SchmittTriggerDetector::SchmittTriggerDetector()
    : m_pimpl(std::make_unique<Impl>())
{
    m_contactStates.clear();
    m_pimpl->manager.clear();
}

SchmittTriggerDetector::~SchmittTriggerDetector() = default;

bool SchmittTriggerDetector::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[SchmittTriggerDetector::initialize]";

    if (m_detectorState != State::NotInitialized)
    {
        log()->error("{} The contact detector already seems to be initialized.", logPrefix);
        return false;
    }

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler", logPrefix);
        return false;
    }

    auto setupParam = [logPrefix, ptr](const std::string& param, auto& vector) -> bool {
        if (!ptr->getParameter(param, vector))
        {
            log()->error("{} Unable to find the parameter named '{}'.", logPrefix, param);
            return false;
        }
        return true;
    };

    std::vector<std::string> contacts;
    bool ok = setupParam("contacts", contacts);

    std::vector<double> onThreshold;
    ok = ok && setupParam("contact_make_thresholds", onThreshold);

    std::vector<double> offThreshold;
    ok = ok && setupParam("contact_break_thresholds", offThreshold);

    std::vector<std::chrono::nanoseconds> switchOnAfter;
    ok = ok && setupParam("contact_make_switch_times", switchOnAfter);

    std::vector<std::chrono::nanoseconds> switchOffAfter;
    ok = ok && setupParam("contact_break_switch_times", switchOffAfter);

    if (!ok)
    {
        return false;
    }

    if ((contacts.size() != onThreshold.size()) || (contacts.size() != offThreshold.size())
        || (contacts.size() != switchOnAfter.size()) || (contacts.size() != switchOffAfter.size()))
    {
        log()->error("{} Configuration parameters size mismatch.", logPrefix);
        return false;
    }

    // initialize the smith trigger for each contact
    for (std::size_t idx = 0; idx < contacts.size(); idx++)
    {
        blf::Math::SchmittTrigger::Params params;
        params.switchOnAfter = switchOnAfter[idx];
        params.switchOffAfter = switchOffAfter[idx];
        params.onThreshold = onThreshold[idx];
        params.offThreshold = offThreshold[idx];

        // set the initial state for the trigger
        constexpr blf::Math::SchmittTriggerState initialState{false,
                                                              std::chrono::nanoseconds::zero(),
                                                              std::chrono::nanoseconds::zero()};
        if (!this->addContact(contacts[idx], initialState, params))
        {
            log()->error("{} Could not add Schmitt Trigger unit for specified contact.", logPrefix);
            return false;
        }
    }

    m_detectorState = State::Initialized;

    return true;
}

bool SchmittTriggerDetector::advance()
{
    constexpr auto logPrefix = "[SchmittTriggerDetector::advance]";

    if (m_detectorState == State::NotInitialized)
    {
        log()->error("{} Please initialize the contact detector before running advance.",
                     logPrefix);
        return false;
    }

    if (m_detectorState == State::Initialized)
    {
        m_detectorState = State::Running;
    }

    for (auto& [contactName, schmittTrigger] : m_pimpl->manager)
    {
        // advance the trigger and get the state
        if (!schmittTrigger.advance())
        {
            log()->error("{} Unable to update the state of the Schmitt trigger cell for the "
                         "contact named: '{}'.",
                         logPrefix,
                         contactName);
            return false;
        }

        // update the contact
        const blf::Math::SchmittTriggerOutput& output = schmittTrigger.getOutput();
        blf::Contacts::EstimatedContact& contact = m_contactStates.at(contactName);
        contact.isActive = output.state;
        contact.switchTime = output.switchTime;
    }

    return true;
}

bool SchmittTriggerDetector::setTimedTriggerInput(const std::string& contactName,
                                                  const blf::Math::SchmittTriggerInput& input)
{
    constexpr auto logPrefix = "[SchmittTriggerDetector::setTimedTriggerInput]";
    if (!m_pimpl->contactExists(contactName))
    {
        log()->error("{} Contact does not exist. Cannot set measurement", logPrefix);
        return false;
    }

    return m_pimpl->manager.at(contactName).setInput(input);
}

bool SchmittTriggerDetector::setTimedTriggerInputs(
    const std::unordered_map<std::string, blf::Math::SchmittTriggerInput>& timedInputs)
{
    std::string_view logPrefix = "[SchmittTriggerDetector::setTimedTriggerInputs] ";

    std::string skippedUpdatesFrames;
    for (const auto& [contactName, measure] : timedInputs)
    {
        if (!m_pimpl->contactExists(contactName))
        {
            skippedUpdatesFrames += " " + contactName;
            continue;
        }

        m_pimpl->manager.at(contactName).setInput(measure);
    }

    if (skippedUpdatesFrames.size())
    {
        log()->debug("{} Skipped setting measurements for contact that does not exist:{}.",
                     logPrefix,
                     skippedUpdatesFrames);
    }

    return true;
}

bool SchmittTriggerDetector::addContact(const std::string& contactName,
                                        const blf::Math::SchmittTriggerState& initialState,
                                        const blf::Math::SchmittTrigger::Params& params)
{
    constexpr auto logPrefix = "[SchmittTriggerDetector::addContact]";

    if (m_pimpl->contactExists(contactName))
    {
        log()->error("{} Contact already exists.", logPrefix);
        return false;
    }

    EstimatedContact newContact;
    newContact.isActive = initialState.state;
    newContact.name = contactName;

    if (!m_pimpl->manager[contactName].initialize(params)
        || !m_pimpl->manager[contactName].setState(initialState))
    {
        log()->error("{} Unable to initialize the trigger named: {}.", logPrefix, contactName);
        m_pimpl->manager.erase(contactName);
        return false;
    }

    std::string contactsKey = contactName;
    m_contactStates.emplace(std::move(contactsKey), std::move(newContact));

    return true;
}

bool SchmittTriggerDetector::removeContact(const std::string& contactName)
{
    constexpr auto logPrefix = "[SchmittTriggerDetector::removeContact]";
    if (!m_pimpl->contactExists(contactName))
    {
        log()->error("{} The contact named '{}' does not exist", logPrefix, contactName);
        return false;
    }

    m_pimpl->manager.erase(contactName);
    m_contactStates.erase(contactName);
    return true;
}

bool SchmittTriggerDetector::resetContact(const std::string& contactName,
                                          const bool state,
                                          const blf::Math::SchmittTrigger::Params& params)
{
    constexpr auto logPrefix = "[SchmittTriggerDetector::resetContact]";
    if (!m_pimpl->contactExists(contactName))
    {
        log()->error("{} The contact named '{}' does not exist", logPrefix, contactName);
        return false;
    }

    auto& trigger = m_pimpl->manager.at(contactName);
    if (!trigger.initialize(params))
    {
        log()->error("{} Unable to initialize the trigger for the contact named '{}'.",
                     logPrefix,
                     contactName);
        return false;
    }
    blf::Math::SchmittTriggerState triggerState;
    triggerState.state = state;
    trigger.setState(triggerState);
    m_contactStates.at(contactName).isActive = state;
    m_contactStates.at(contactName).switchTime = std::chrono::nanoseconds::zero();

    return true;
}

bool SchmittTriggerDetector::resetState(const std::string& contactName, const bool& state)
{
    constexpr auto logPrefix = "[SchmittTriggerDetector::resetContact]";
    if (!m_pimpl->contactExists(contactName))
    {
        log()->error("{} The contact named '{}' does not exist", logPrefix, contactName);
        return false;
    }

    blf::Math::SchmittTriggerState triggerState;
    triggerState.state = state;
    m_pimpl->manager.at(contactName).setState(triggerState);
    m_contactStates.at(contactName).isActive = state;
    m_contactStates.at(contactName).switchTime = std::chrono::nanoseconds::zero();

    return true;
}
