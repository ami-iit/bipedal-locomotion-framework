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

/**
 * @brief Utility struct to store a reference to a contact.
 */
struct ContactReference
{
    std::string listLabel; /** Label to indicate the corresponding list. **/
    ContactList::const_iterator contact_it; /** Const iterator to a contact. **/
};

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
    std::vector<ContactReference> activeContacts;

    /**
     * @brief Utility function to check if a list is present amongst the active contacts.
     * @param key The label of the list to be checked.
     * @return True if key is present amongst the active contacts.
     **/
    bool isListIncluded(const std::string& key) const;

    /**
     * @brief Utility function to retrieve the an iterator to an active contact given the list label
     * @param key The label of interest
     * @return An iterator to the element inside activeContacts containing the desired key.
     * If no contact with the desired label is present, it returns an iterator to the end of the vector.
     */
    std::vector<ContactReference>::const_iterator getContactGivenList(const std::string& key) const;
};

}
}

#endif // BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_PHASE_H
