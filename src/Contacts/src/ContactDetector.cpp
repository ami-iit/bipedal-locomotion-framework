/**
 * @file ContactDetector.cpp
 * @authors Prashanth Ramadoss, Giulio Romualdi
 * @copyright 2020-2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContactDetectors/ContactDetector.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Contacts;

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
    auto iterator = m_contactStates.find(contactName);
    if (iterator == m_contactStates.end())
    {
        log()->error("[ContactDetector::get] Contact not found.");
        return false;
    }

    contact = iterator->second;
    return true;
}

EstimatedContact ContactDetector::get(const std::string& contactName) const
{
    auto iterator = m_contactStates.find(contactName);
    if (iterator == m_contactStates.end())
    {
        log()->error("[ContactDetector::get] Contact not found.");
        return EstimatedContact();
    }

    return iterator->second;
}

bool ContactDetector::isOutputValid() const
{
    return m_detectorState == State::Running;
}
