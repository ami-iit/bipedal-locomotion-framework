/**
 * @file ContactPhase.h
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_PHASE_LIST_H
#define BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_PHASE_LIST_H

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Contacts/ContactPhase.h>

#include <unordered_map>
#include <initializer_list>
#include <vector>

namespace BipedalLocomotion
{
namespace Contacts
{
/**
 * @brief Utility alias to a map of ContacLists.
 */
using ContactListMap = std::unordered_map<std::string, ContactList>;

/**
 * @brief The ContactPhaseList class computes the contact phases according to a bunch of input contact lists.
 * @warning All the iterators stored inside the contact phases refer to the lists stored within this class, not the original input lists.
 */
class ContactPhaseList
{

    ContactListMap m_contactLists; /** The input contact lists. **/

    std::vector<ContactPhase> m_phases; /** The computed phases. **/

    void createPhases(); /** Internal method to compute the phases. **/

public:

    using const_iterator = std::vector<ContactPhase>::const_iterator;
    using const_reverse_iterator = std::vector<ContactPhase>::const_reverse_iterator;

    /**
     * @brief Set the input lists
     * @param contactLists The set of lists to be used for computing the phases.
     */
    void setLists(const ContactListMap& contactLists);

    /**
     * @brief Set the input lists
     * @param An initializer list (use as {list1, list2, ...,listN}) to the lists to be used for computing the phases.
     * A ContactListMap will be created with the provided list, using the defaultName as a key.
     * @return False if some lists have the same defaultName.
     */
    bool setLists(const std::initializer_list<ContactList>& contactLists);

    /**
     * @brief A reference to the lists stored in this class.
     * @warning All the iterators stored inside the contact phases refer to the lists viewable via this method.
     * @return A const reference to the input lists.
     */
    const ContactListMap& lists() const;

    /**
     * @brief Const iterator to the begin of the phases.
     */
    const_iterator begin() const;

    /**
     * @brief Const iterator to the begin of the phases.
     */
    const_iterator cbegin() const;

    /**
     * @brief Const reverse iterator to the the phases (basically starting from the last phase, going backward).
     */
    const_reverse_iterator rbegin() const;

    /**
     * @brief Const reverse iterator to the the phases (basically starting from the last phase, going backward).
     */
    const_reverse_iterator crbegin() const;

    /**
     * @brief Return a const iterator to the end of the list.
     *
     * This is only a placeholder, it does not reference any phase.
     */
    const_iterator end() const;

    /**
     * @brief Return a const iterator to the end of the list.
     *
     * This is only a placeholder, it does not reference any phase.
     */
    const_iterator cend() const;

    /**
     * @brief Return a const reverse iterator to the end of the list.
     *
     * This is only a placeholder, it does not reference any phase.
     */
    const_reverse_iterator rend() const;

    /**
     * @brief Return a const reverse iterator to the end of the list.
     *
     * This is only a placeholder, it does not reference any phase.
     */
    const_reverse_iterator crend() const;

    /**
     * @brief Access phases by index.
     * @param index of the phase to be accessed.
     * @return A const reference to the desired phase.
     */
    const ContactPhase& operator[](size_t index) const;

    /**
     * @brief A const iterator to the first phase.
     */
    const_iterator firstPhase() const;

    /**
     * @brief A const iterator to the last phase.
     */
    const_iterator lastPhase() const;

    /**
     * @brief Get the number of phases.
     */
    size_t size() const;

    /**
     * @brief Clear the phases and the stored lists.
     */
    void clear();
};

}
}

#endif // BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_PHASE_LIST_H
