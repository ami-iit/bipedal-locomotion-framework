/**
 * @file ContactDetector.cpp
 * @authors Prashanth Ramadoss, Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/ContactDetectors/ContactDetector.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Contacts;

bool ContactDetector::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto printPrefix = "[ContactDetector::initialize]";
    if (m_detectorState != State::NotInitialized)
    {
        log()->error("{} The contact detector already seems to be initialized.", printPrefix);
        return false;
    }

    auto handle = handler.lock();
    if (handle == nullptr)
    {
        log()->error("{} The parameter handler has expired. Please check its scope.", printPrefix);
        return false;
    }

    if (!this->customInitialization(handler))
    {
        log()->error("{} Could not run custom initialization of the contact detector.",
                     printPrefix);
        return false;
    }

    log()->info("{} Contact detector initialization is successful.", printPrefix);

    m_detectorState = State::Initialized;
    return true;
}

bool ContactDetector::customInitialization(std::weak_ptr<const IParametersHandler> handler)
{
    return true;
}

bool ContactDetector::advance()
{
    constexpr auto printPrefix = "[ContactDetector::advance]";
    if (m_detectorState == State::NotInitialized)
    {
        log()->error("{} Please initialize the contact detector before running advance.",
                     printPrefix);
        return false;
    } else
    {
        m_detectorState = State::Running;
    }

    if (!this->updateContactStates())
    {
        log()->error("{} Unable to update the contact state.", printPrefix);
        return false;
    }
    return true;
}

bool ContactDetector::resetContacts()
{
    for (auto& [name, contact] : m_contactStates)
    {
        contact.switchTime = 0.0;
        contact.isActive = false;
    }
    return true;
}

const EstimatedContactList& ContactDetector::getOutput() const
{
    return m_contactStates;
}

bool ContactDetector::get(const std::string& contactName, EstimatedContact& contact) const
{
    if (m_contactStates.find(contactName) == m_contactStates.end())
    {
        log()->error("[ContactDetector::get] Contact not found.");
        return false;
    }

    contact = m_contactStates.at(contactName);
    return true;
}

EstimatedContact ContactDetector::get(const std::string& contactName) const
{
    if (m_contactStates.find(contactName) == m_contactStates.end())
    {
        log()->error("[ContactDetector::get] Contact not found.");
        return EstimatedContact();
    }

    return m_contactStates.at(contactName);
}

bool ContactDetector::isOutputValid() const
{
    return (m_detectorState == State::Running);
}
