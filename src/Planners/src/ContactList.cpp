/**
 * @file ContactList.cpp
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Planners/ContactList.h>
#include <iostream>

using namespace BipedalLocomotion::Planners;

void ContactList::setDefaultName(const std::string &defaultName)
{
    m_defaultName = defaultName;
}

const std::string &ContactList::defaultName() const
{
    return m_defaultName;
}

void ContactList::setDefaultContactType(const ContactType &type)
{
    m_defaultContactType = type;
}

const ContactType &ContactList::defaultContactType() const
{
    return m_defaultContactType;
}

bool ContactList::addContact(const Contact &newContact)
{
    if (newContact.activationTime > newContact.deactivationTime)
    {
        std::cerr << "[ContactList::addContact] The activation time cannot be greater than the deactivation time." <<std::endl;
        return false;
    }
    using iterator = std::set<Contact, ContactCompare>::iterator;
    std::pair<iterator,bool> res = m_contacts.insert(newContact);
    if (!res.second)
    {
        std::cerr << "[ContactList::addContact] Failed to insert new element.";
        if (res.first != end())
        {
            std::cerr << " The new contact (activationTime: " << newContact.activationTime << " ";
            std::cerr << "deactivationTime: " << newContact.deactivationTime << ") is not compatible";
            std::cerr << " with an element already present in the list (activationTime: " << res.first->activationTime << " ";
            std::cerr << "deactivationTime: " << res.first->deactivationTime << ")" << std::endl;
        }
        return false;
    }
    return true;
}

bool ContactList::addContact(const iDynTree::Transform &newTransform, double activationTime, double deactivationTime)
{
    Contact newContact;
    newContact.pose = newTransform;
    newContact.activationTime = activationTime;
    newContact.deactivationTime = deactivationTime;
    newContact.name = m_defaultName;
    newContact.type = m_defaultContactType;

    return addContact(newContact);
}

ContactList::const_iterator ContactList::erase(const_iterator iterator)
{
    return m_contacts.erase(iterator);
}

ContactList::const_iterator ContactList::begin() const
{
    return m_contacts.begin();
}

ContactList::const_iterator ContactList::cbegin() const
{
    return m_contacts.cbegin();
}

ContactList::const_reverse_iterator ContactList::rbegin() const
{
    return m_contacts.rbegin();
}

ContactList::const_reverse_iterator ContactList::crbegin() const
{
    return m_contacts.crbegin();
}

ContactList::const_iterator ContactList::end() const
{
    return m_contacts.end();
}

ContactList::const_iterator ContactList::cend() const
{
    return m_contacts.cend();
}

ContactList::const_reverse_iterator ContactList::rend() const
{
    return m_contacts.rend();
}

ContactList::const_reverse_iterator ContactList::crend() const
{
    return m_contacts.crend();
}

size_t ContactList::size() const
{
    return m_contacts.size();
}

ContactList::const_iterator ContactList::firstStep() const
{
    return begin();
}

ContactList::const_iterator ContactList::lastStep() const
{
    return --end();
}

bool ContactList::editContact(ContactList::const_iterator element, const Contact &newContact)
{
    if (element == end())
    {
        std::cerr << "[ContactList::addContact] The element is not valid." << std::endl;
        return false;
    }

    ContactList::const_iterator previousElement = element;
    ContactList::const_iterator nextElement = element;
    nextElement++;
    if (element != begin())
    {
        --previousElement;
        if (newContact.activationTime < previousElement->deactivationTime)
        {
            std::cerr << "[ContactList::addContact] The new contact cannot have an activation time smaller than the previous contact." << std::endl;
            return false;
        }
    }

    if (nextElement != end())
    {
        if (newContact.deactivationTime > nextElement->activationTime)
        {
            std::cerr << "[ContactList::addContact] The new contact cannot have a deactivation time greater than the next contact." << std::endl;
            return false;
        }
    }

    m_contacts.erase(element);
    m_contacts.insert(nextElement, newContact);

    return true;
}

ContactList::const_iterator ContactList::getPresentStep(double time) const
{
    // With the reverse iterator we find the last step such that the activation time is smaller that time
    ContactList::const_reverse_iterator presentReverse = std::find_if(rbegin(), rend(),
                                                        [time](const Contact & a) -> bool { return a.activationTime <= time; });
    if (presentReverse == rend())
    {
        // No contact has activation time lower than the specified time.
        return end();
    }

    return --(presentReverse.base()); //This is to convert a reverse iterator to a forward iterator. The -- is because base() returns a forward iterator to the next element.
}

bool ContactList::keepOnlyPresentStep(double time)
{
    ContactList::const_iterator dropPoint = getPresentStep(time);

    if (dropPoint == end())
    {
        std::cerr << "[ContactList::addContact] No contact has activation time lower than the specified time." << std::endl;
        return false;
    }

    Contact present = *dropPoint;

    clear();
    addContact(present);

    return true;
}

void ContactList::clear()
{
    m_contacts.clear();
}

void ContactList::removeLastStep()
{
    erase(lastStep());
}

