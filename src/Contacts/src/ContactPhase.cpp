/**
 * @file ContactPhase.cpp
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Contacts/ContactPhase.h>
#include <algorithm>

using namespace BipedalLocomotion::Contacts;

bool ContactPhase::isListIncluded(const std::string& key) const
{
    return activeContacts.find(key) != activeContacts.end();
}

std::string ContactPhase::toString() const
{
    std::stringstream ss;
    ss << "Phase start time: "
       << std::chrono::duration_cast<std::chrono::milliseconds>(beginTime).count()
       << "ms end time: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTime).count()
       << "ms active contacts: ";
    for (const auto& [key, contact] : activeContacts)
    {
        ss << key << " " << contact->toString() << " ";
    }
    return ss.str();
}
