/**
 * @file ContactList.h
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_CONTACTLIST_H
#define BIPEDAL_LOCOMOTION_PLANNERS_CONTACTLIST_H

#include <BipedalLocomotion/Planners/Contact.h>
#include <iDynTree/Core/Transform.h>
#include <set>
#include <string>
#include <functional>

namespace BipedalLocomotion
{
namespace Planners
{

/**
 * @brief Class containing a list of steps.
 * The steps are added such that the activation time is strictly growing.
 * It represents a series of contact activations and deactivations of a single entity.
 * Once items are inserted, items cannot change since they are ordered according to their timing.
 */
class ContactList
{
    struct ContactCompare {
        bool operator()(const Contact& lhs, const Contact& rhs) const {
            return lhs.deactivationTime < rhs.activationTime;
        }
    };

    std::set<Contact, ContactCompare> m_contacts;
    std::string m_defaultName{"ContactList"};
    ContactType m_defaultContactType{ContactType::FULL};

public:

    using const_iterator = std::set<Contact, ContactCompare>::const_iterator;
    using const_reverse_iterator = std::set<Contact, ContactCompare>::const_reverse_iterator;

    void setDefaultName(const std::string& defaultName);

    const std::string& defaultName() const;

    void setDefaultContactType(const ContactType& type);

    const ContactType& defaultContactType() const;

    bool addContact(const Contact& newContact);

    bool addContact(const iDynTree::Transform& newTransform, double activationTime, double deactivationTime);

    const_iterator erase(const_iterator iterator);

    const_iterator begin() const;
    const_iterator cbegin() const;

    const_reverse_iterator rbegin() const;
    const_reverse_iterator crbegin() const;

    const_iterator end() const;
    const_iterator cend() const;

    const_reverse_iterator rend() const;
    const_reverse_iterator crend() const;

    size_t size() const;

    const_iterator firstStep() const;

    const_iterator lastStep() const;

    bool editContact(const_iterator element, const Contact& newContact);

    const_iterator getPresentStep(double time) const;

    bool keepOnlyPresentStep(double time);

    void clear();

    void removeLastStep();

};

} //namespace Planners
} //namespace BipedalLocomotion



#endif // BIPEDAL_LOCOMOTION_PLANNERS_CONTACTLIST_H
