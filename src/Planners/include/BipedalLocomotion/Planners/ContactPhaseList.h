/**
 * @file ContactPhase.h
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_PHASE_LIST_H
#define BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_PHASE_LIST_H

#include <BipedalLocomotion/Planners/Contact.h>
#include <BipedalLocomotion/Planners/ContactList.h>
#include <BipedalLocomotion/Planners/ContactPhase.h>

#include <unordered_map>
#include <initializer_list>
#include <vector>

namespace BipedalLocomotion
{
namespace Planners
{

using ContactListMap = std::unordered_map<std::string, ContactList>;

class ContactPhaseList
{

    ContactListMap m_contactLists;

    std::vector<ContactPhase> m_phases;

    void createPhases();

public:

    using const_iterator = std::vector<ContactPhase>::const_iterator;
    using const_reverse_iterator = std::vector<ContactPhase>::const_reverse_iterator;

    void setLists(const ContactListMap& contactLists);

    bool setLists(const std::initializer_list<ContactList>& contactLists);

    const ContactListMap& lists() const;

    const_iterator begin() const;
    const_iterator cbegin() const;

    const_reverse_iterator rbegin() const;
    const_reverse_iterator crbegin() const;

    const_iterator end() const;
    const_iterator cend() const;

    const_reverse_iterator rend() const;
    const_reverse_iterator crend() const;

    const ContactPhase& operator[](size_t index) const;

    const_iterator firstPhase() const;

    const_iterator lastPhase() const;

    size_t size() const;

    void clear();
};

}
}

#endif // BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_PHASE_LIST_H
