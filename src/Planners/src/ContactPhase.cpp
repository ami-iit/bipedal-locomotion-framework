/**
 * @file ContactPhase.cpp
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Planners/ContactPhase.h>
#include <algorithm>

using namespace BipedalLocomotion::Planners;

bool ContactPhase::isListIncluded(const std::string &key) const
{
    std::vector<ContactReference>::const_iterator it = getContactGivenList(key);

    return  it != activeContacts.end();
}

std::vector<ContactReference>::const_iterator ContactPhase::getContactGivenList(const std::string &key) const
{
    return std::find_if(activeContacts.begin(), activeContacts.end(),
                        [key](const ContactReference& ref){return ref.listLabel == key;});
}
