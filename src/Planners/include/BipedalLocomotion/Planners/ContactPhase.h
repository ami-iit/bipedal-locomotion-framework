/**
 * @file ContactPhase.h
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_PHASE_H
#define BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_PHASE_H

#include <BipedalLocomotion/Planners/Contact.h>
#include <BipedalLocomotion/Planners/ContactList.h>
#include <vector>
#include <string>

namespace BipedalLocomotion
{
namespace Planners
{

struct ContactReference
{
    std::string listLabel;
    ContactList::const_iterator contact_it;
};

struct ContactPhase
{
    double beginTime {0.0};

    double endTime {0.0};

    std::vector<ContactReference> activeContacts;

    bool isListIncluded(const std::string& key) const;

    std::vector<ContactReference>::const_iterator getContactGivenList(const std::string& key) const;
};

}
}

#endif // BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_PHASE_H
