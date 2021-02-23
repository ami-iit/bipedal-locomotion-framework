/**
 * @file ContactPhase.cpp
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Contacts/ContactPhase.h>
#include <algorithm>

using namespace BipedalLocomotion::Contacts;

bool ContactPhase::isListIncluded(const std::string &key) const
{
    return  activeContacts.find(key) != activeContacts.end();
}
