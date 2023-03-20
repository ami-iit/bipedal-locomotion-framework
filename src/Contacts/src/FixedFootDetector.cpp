/**
 * @file FixedFootDetector.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContactDetectors/FixedFootDetector.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <limits>
#include <memory>

using namespace BipedalLocomotion::Contacts;
using namespace BipedalLocomotion::ParametersHandler;

bool FixedFootDetector::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[FixedFootDetector::customInitialization]";

    if (m_detectorState != State::NotInitialized)
    {
        log()->error("{} The contact detector already seems to be initialized.", logPrefix);
        return false;
    }

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler has expired. Please check its scope.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("sampling_time", m_dT))
    {
        log()->error("{} Unable to find the 'sampling_time' parameter.", logPrefix);
        return false;
    }

    if (m_dT <= 0)
    {
        log()->error("{} The parameter 'sampling_time' must be a strictly positive number.",
                     logPrefix);
        return false;
    }

    m_detectorState = State::Initialized;

    return true;
}

bool FixedFootDetector::updateFixedFoot()
{
    constexpr auto logPrefix = "[FixedFootDetector::updateFixedFoot]";

    // search the phase associated to the current time
    auto phase = m_contactPhaselist.getPresentPhase(m_currentTime);

    if(phase == m_contactPhaselist.end())
    {
        log()->error("{} No phase has the begin time lower than the specified time.", logPrefix);
        return false;
    }

    if (phase->activeContacts.size() > 2)
    {
        log()->error("{} The base detector can be used only for bipedal locomotion and it cannot "
                     "handle more than two actives contacts.",
                     logPrefix);
        return false;
    }

    // if the robot is in single support only one contact is enabled
    if (phase->activeContacts.size() == 1)
    {
        // get the enabled contact
        const auto it = phase->activeContacts.cbegin();
        if (m_contactStates.find(it->first) == m_contactStates.end())
        {
            log()->error("{} Unable to find the contact.", logPrefix);
            return false;
        }

        // update the contacts
        for (auto& [name, contact] : m_contactStates)
        {
            contact.lastUpdateTime = m_currentTime;
            contact.switchTime = phase->beginTime;
            if (name != it->first)
            {
                contact.isActive = false;
            } else
            {
                contact.isActive = true;
                contact.pose = it->second->pose;
            }
        }

        return true;
    }

    // the active contacts are 2
    if (phase->activeContacts.size() != 2)
    {
        log()->error("{} This class supports only the bipedal robots where at list one foot is in "
                     "contact with the ground. This means that the maximum number of active "
                     "contact must be equal to 1 or 2. In the current phase the number of active "
                     "contacts is equal to {}.",
                     logPrefix,
                     phase->activeContacts.size());
        return false;
    }

    // Notice that here we do not analyze the case in which the phase is different from
    // firstPhase. Indeed in this case we should not update the contact states dictionary. (The
    // content of the dictionary is the one of the previous single support phase)
    if (phase != m_contactPhaselist.firstPhase())
    {
        return true;
    }

    // if the current phase is the first and there are at least 2 phases the active contact will be
    // the one that is going to be active in the next phase.
    if (m_contactPhaselist.size() > 1)
    {
        const auto nextPhase = std::next(phase);
        const auto it = nextPhase->activeContacts.cbegin();

        assert(m_contactStates.find(it->first) != m_contactStates.end()
               && "[FixedFootDetector::updateFixedFoot] Unable to find the contact. "
                  "This should not be possible. If you read this assert there is a bug in "
                  "the code");

        // update the contacts
        for (auto& [name, contact] : m_contactStates)
        {
            contact.lastUpdateTime = m_currentTime;
            contact.switchTime = phase->beginTime;
            if (name != it->first)
            {
                contact.isActive = true;
                contact.pose = phase->activeContacts.find(name)->second->pose;
            } else
            {
                contact.isActive = false;
            }
        }
    } else // contactPhaselist.size() == 1
    {
        // there is only one contact phase and this phase is a double support phase.
        // Here we take the first contact as active contact.
        auto contact = phase->activeContacts.cbegin();

        assert(m_contactStates.find(contact->first) != m_contactStates.end()
               && "[FixedFootDetector::updateFixedFoot] Unable to find the contact. "
                  "This should not be possible. If you read this assert there is a bug in "
                  "the code");

        m_contactStates[contact->first].lastUpdateTime = m_currentTime;
        m_contactStates[contact->first].switchTime = phase->beginTime;
        m_contactStates[contact->first].isActive = true;
        m_contactStates[contact->first].pose = contact->second->pose;

        std::advance(contact, 1);

        assert(m_contactStates.find(contact->first) != m_contactStates.end()
               && "[FixedFootDetector::updateFixedFoot] Unable to find the contact. "
                  "This should not be possible. If you read this assert there is a bug in "
                  "the code");

        m_contactStates[contact->first].lastUpdateTime = m_currentTime;
        m_contactStates[contact->first].switchTime = phase->beginTime;
        m_contactStates[contact->first].isActive = false;
    }

    return true;
}

bool FixedFootDetector::advance()
{
    constexpr auto logPrefix = "[FixedFootDetector::advance]";

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

    if (!this->updateFixedFoot())
    {
        return false;
    }

    // if everything went well advance the time
    m_currentTime += m_dT;
    return true;
}

void FixedFootDetector::setContactPhaseList(const ContactPhaseList& phaseList)
{
    m_contactPhaselist = phaseList;

    std::vector<std::string> removedContacts;
    std::vector<std::string> newContacts;

    // check if some contacts have been removed
    for (const auto& [key, value] : m_contactStates)
    {
        if (phaseList.lists().find(key) == phaseList.lists().cend())
        {
            removedContacts.push_back(key);
        }
    }

    // check if some contacts have been added
    for (const auto& [key, value] : phaseList.lists())
    {
        if (m_contactStates.find(key) == m_contactStates.cend())
        {
            newContacts.push_back(key);
        }
    }

    // removed contacts
    for (const auto& contact : removedContacts)
    {
        m_contactStates.erase(std::string(contact));
    }

    // new contacts
    for (const auto& contact : newContacts)
    {
        m_contactStates[contact].name = contact;
        m_contactStates[contact].index = phaseList.lists().find(contact)->second.cbegin()->index;
    }
}

void FixedFootDetector::resetTime(const double &time)
{
    m_currentTime = time;
}

const EstimatedContact& FixedFootDetector::getFixedFoot() const
{
    constexpr auto logPrefix = "[FixedFootDetector::getFixedFoot]";

    // only one foot can be considered as fixed foot
    for (const auto& [key, contact] : m_contactStates)
    {
        if (contact.isActive)
        {
            return contact;
        }
    }

    log()->error("{} Unable to find the fixed foot. This should never happen.", logPrefix);
    return m_dummyContact;
}
