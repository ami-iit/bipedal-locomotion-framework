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
#include <unordered_map>
#include <string>

namespace BipedalLocomotion
{
namespace Planners
{

/**
 * @brief Struct defining a contact phase.
 * Each phase is characterized by a set of contacts which remain active for the entirety of the phase.
 */
struct ContactPhase
{
    /**
     * @brief The phase initial time.
     **/
    double beginTime {0.0};

    /**
     * @brief The phase end time.
     **/
    double endTime {0.0};

    /**
     * @brief The set of contacts active during the phase.
     */
    std::unordered_map<std::string, ContactList::const_iterator> activeContacts;

    /**
     * @brief Utility function to check if a list is present amongst the active contacts.
     * @param key The label of the list to be checked.
     * @return True if key is present amongst the active contacts.
     **/
    bool isListIncluded(const std::string& key) const;
};

}
}

#endif // BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_PHASE_H
