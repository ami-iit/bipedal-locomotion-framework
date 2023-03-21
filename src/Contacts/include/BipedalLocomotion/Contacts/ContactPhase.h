/**
 * @file ContactPhase.h
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_PHASE_H
#define BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_PHASE_H

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Contacts/ContactList.h>

#include <chrono>
#include <unordered_map>
#include <string>

namespace BipedalLocomotion
{
namespace Contacts
{

/**
 * @brief Struct defining a contact phase.
 * Each phase is characterized by a set of contacts which remain active for the entirety of the phase.
 * @note Mathematically speaking the interval of the phase is defined as following
 * \f[
 * I = [t_b \; t_e)
 * \f]
 * where \f$t_b\f$ is the ContactPhase::beginTime and \f$t_e\f$ is the ContactPhase::endTime.
 * The end time is not included in the phase.
 */
struct ContactPhase
{
    /**
     * @brief The phase initial time.
     **/
    std::chrono::nanoseconds beginTime {std::chrono::nanoseconds::zero()};

    /**
     * @brief The phase end time.
     **/
    std::chrono::nanoseconds endTime {std::chrono::nanoseconds::zero()};

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

#endif // BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_PHASE_H
