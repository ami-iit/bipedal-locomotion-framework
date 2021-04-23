/**
 * @file BaseDetector.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/ContactDetectors/BaseDetector.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <memory>

using namespace BipedalLocomotion::Contacts;
using namespace BipedalLocomotion::ParametersHandler;

bool BaseDetector::customInitialization(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[BaseDetector::customInitialization]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler has expired. Please check its scope.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("sampling_time", m_dT))
    {
        log()->error("{} Unable to find the sampling time.", logPrefix);
        return false;
    }

    if (m_dT <= 0)
    {
        log()->error("{} The sampling time must be a strictly positive number.", logPrefix);
        return false;
    }

    return true;
}

bool BaseDetector::updateContactStates()
{
    assert(m_currentTime >= m_contactPhaselist.firstPhase()->beginTime
           && "[BaseDetector::updateContactStates] The current time must be greater than equal "
              "the first contact phase. If you read this assert there is a bug in the code");

    // search the phase associated to the current time
    auto phase = std::find_if(m_contactPhaselist.cbegin(),
                              m_contactPhaselist.cend(),
                              [&](const auto& phase) -> bool {
                                  return phase.beginTime >= m_currentTime
                                         && phase.endTime <= m_currentTime;
                              });

    // take the last phase if not found
    if (phase == m_contactPhaselist.cend())
        phase = m_contactPhaselist.lastPhase();

    // if the robot is in single support only once contact is enabled
    if (phase->activeContacts.size() == 1)
    {
        // get the enabled contact
        const auto it = phase->activeContacts.cbegin();
        assert(m_contactStates.find(it->first) != m_contactStates.end()
               && "[BaseDetector::updateContactStates] Unable to find the contact. This should not "
                  "be possible. If you read this assert there is a bug in the code");

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
    } else if (phase->activeContacts.size() == 2)
    {
        // Notice that here we do not analyze the case in which the phase is differnt from
        // firstPhase. Indeed in this case we should not update the contact states dictionary. (The
        // contet of the dictionary is the one of the previous single support phase)

        // if the current phase is the first and there are at least 2 phases the active contact will
        // be the one that in the next phase is active
        if (phase == m_contactPhaselist.firstPhase() && m_contactPhaselist.size() > 1)
        {
            const auto nextPhase = std::next(phase);
            const auto it = nextPhase->activeContacts.cbegin();

            assert(m_contactStates.find(it->first) != m_contactStates.end()
                   && "[BaseDetector::updateContactStates] Unable to find the contact. This should "
                      "not "
                      "be possible. If you read this assert there is a bug in the code");

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
        } else
        {
            // there is only one contact phase and this phase is a double support phase.
            // Here we take the first contact as active contact.
            auto contact = phase->activeContacts.cbegin();

            assert(m_contactStates.find(contact->first) != m_contactStates.end()
                   && "[BaseDetector::updateContactStates] Unable to find the contact. This should "
                      "not "
                      "be possible. If you read this assert there is a bug in the code");

            m_contactStates[contact->first].lastUpdateTime = m_currentTime;
            m_contactStates[contact->first].switchTime = phase->beginTime;
            m_contactStates[contact->first].isActive = true;
            m_contactStates[contact->first].pose = contact->second->pose;

            std::advance(contact, 1);

            assert(m_contactStates.find(contact->first) != m_contactStates.end()
                   && "[BaseDetector::updateContactStates] Unable to find the contact. This should "
                      "not "
                      "be possible. If you read this assert there is a bug in the code");

            m_contactStates[contact->first].lastUpdateTime = m_currentTime;
            m_contactStates[contact->first].switchTime = phase->beginTime;
            m_contactStates[contact->first].isActive = false;
        }
    } else
    {
        log()->error("[BaseDetector::updateContactStates] The base detector can be used only for "
                     "bipedal locomotion and it cannot handle more than two actives contacts.");
        return false;
    }

    m_currentTime += m_dT;
    return true;
}

void BaseDetector::setContactPhaseList(const Contacts::ContactPhaseList& phaseList)
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
    }

    // set the initial time
    m_currentTime = phaseList.firstPhase()->beginTime;
}
