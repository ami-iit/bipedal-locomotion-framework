/**
 * @file ContactList.cpp
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <cassert>
#include <iterator>

using namespace BipedalLocomotion::Contacts;

bool ContactList::ContactCompare::operator()(const PlannedContact& lhs,
                                             const PlannedContact& rhs) const
{
    return lhs.deactivationTime < rhs.activationTime;
}

void ContactList::setDefaultName(const std::string& defaultName)
{
    m_defaultName = defaultName;
}

const std::string& ContactList::defaultName() const
{
    return m_defaultName;
}

void ContactList::setDefaultContactType(const ContactType& type)
{
    m_defaultContactType = type;
}

const ContactType& ContactList::defaultContactType() const
{
    return m_defaultContactType;
}

void ContactList::setDefaultIndex(int defaultIndex)
{
    m_defaultIndex = defaultIndex;
}

int ContactList::defaultIndex() const
{
    return m_defaultIndex;
}

bool ContactList::addContact(const PlannedContact& newContact)
{
    constexpr auto errorPrefix = "[ContactList::addContact]";

    if (newContact.activationTime > newContact.deactivationTime)
    {
        log()->error("{} The activation time cannot be greater than the deactivation time.",
                     errorPrefix);
        return false;
    }
    using iterator = std::set<PlannedContact, ContactCompare>::iterator;
    std::pair<iterator, bool> res = m_contacts.insert(newContact);
    if (!res.second)
    {
        log()->error("{} Failed to insert new element.", errorPrefix);
        if (res.first != end())
        {
            log()->error("{} The new contact (activationTime: {} deactivationTime: {}) is not "
                         "compatible with an element already present in the list (activationTime: "
                         "{}, deactivationTime: {}.).",
                         errorPrefix,
                         newContact.activationTime,
                         newContact.deactivationTime,
                         res.first->activationTime,
                         res.first->deactivationTime);
        }
        return false;
    }
    return true;
}

bool ContactList::addContact(const manif::SE3d& newTransform,
                             const std::chrono::nanoseconds& activationTime,
                             const std::chrono::nanoseconds& deactivationTime)
{
    PlannedContact newContact;
    newContact.pose = newTransform;
    newContact.activationTime = activationTime;
    newContact.deactivationTime = deactivationTime;
    newContact.name = m_defaultName;
    newContact.index = m_defaultIndex;
    newContact.type = m_defaultContactType;

    return this->addContact(newContact);
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

const PlannedContact& ContactList::operator[](size_t index) const
{
    assert(index < size());

    if (index > (size() / 2 + (size() % 2 != 0))) // fancy way for doing std::ceil(size() / 2.0)
    {
        ContactList::const_reverse_iterator it = rbegin();
        std::advance(it, size() - index - 1);
        return *(it);
    } else
    {
        ContactList::const_iterator it = begin();
        std::advance(it, index);
        return *(it);
    }
}

size_t ContactList::size() const
{
    return m_contacts.size();
}

ContactList::const_iterator ContactList::firstContact() const
{
    return begin();
}

ContactList::const_iterator ContactList::lastContact() const
{
    return --end();
}

bool ContactList::editContact(ContactList::const_iterator element, const PlannedContact& newContact)
{
    if (element == end())
    {
        log()->error("[ContactList::editContact] The element is not valid.");
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
            log()->error("[ContactList::editContact] The new contact cannot have an activation "
                         "time "
                         "smaller than the previous contact.");
            return false;
        }
    }

    if (nextElement != end())
    {
        if (newContact.deactivationTime > nextElement->activationTime)
        {
            log()->error("[ContactList::editContact] The new contact cannot have a deactivation "
                         "time greater than the next contact.");
            return false;
        }
    }

    m_contacts.erase(element);
    m_contacts.insert(nextElement, newContact);

    return true;
}

ContactList::const_iterator
ContactList::getPresentContact(const std::chrono::nanoseconds& time) const
{
    // With the reverse iterator we find the last step such that the activation time is smaller
    // equal than time
    ContactList::const_reverse_iterator presentReverse
        = std::find_if(rbegin(), rend(), [time](const PlannedContact& a) -> bool {
              return a.activationTime <= time;
          });

    if (presentReverse == rend())
    {
        // No contact has activation time lower than the specified time.
        return end();
    }

    return --(presentReverse.base()); // This is to convert a reverse iterator to a forward
                                      // iterator. The -- is because base() returns a forward
                                      // iterator to the next element.
}

ContactList::const_iterator
ContactList::getActiveContact(const std::chrono::nanoseconds& time) const
{
    // With the reverse iterator we find the last step such that the activation time is smaller
    // equal than time
    ContactList::const_reverse_iterator presentReverse
        = std::find_if(rbegin(), rend(), [time](const PlannedContact& a) -> bool {
              return a.activationTime <= time && a.deactivationTime > time;
          });

    if (presentReverse == rend())
    {
        // No contact has activation time lower than the specified time.
        return end();
    }

    return --(presentReverse.base()); // This is to convert a reverse iterator to a forward
                                      // iterator. The -- is because base() returns a forward
                                      // iterator to the next element.
}

ContactList::const_iterator ContactList::getNextContact(const std::chrono::nanoseconds& time) const
{
    // We find the last step such that the activation time is smaller equal than time
    ContactList::const_iterator nextContact
        = std::find_if(begin(), end(), [time](const PlannedContact& a) -> bool {
              return a.activationTime > time;
          });

    return nextContact;
}

ContactList::const_iterator
ContactList::getPreviousContact(const std::chrono::nanoseconds& time) const
{
    // With the reverse iterator we find the last step such that the deactivation time is smaller
    // than time
    ContactList::const_reverse_iterator presentReverse
        = std::find_if(rbegin(), rend(), [time](const PlannedContact& a) -> bool {
              return a.deactivationTime < time;
          });

    if (presentReverse == rend())
    {
        // No contact has activation time lower than the specified time.
        return end();
    }

    return --(presentReverse.base()); // This is to convert a reverse iterator to a forward
                                      // iterator. The -- is because base() returns a forward
                                      // iterator to the next element.
}

bool ContactList::keepOnlyPresentContact(const std::chrono::nanoseconds& time)
{
    ContactList::const_iterator dropPoint = getPresentContact(time);

    if (dropPoint == end())
    {
        log()->error("[ContactList::addContact] No contact has activation time lower than the "
                     "specified time.");
        return false;
    }

    PlannedContact present = *dropPoint;

    clear();
    addContact(present);

    return true;
}

void ContactList::clear()
{
    m_contacts.clear();
}

void ContactList::removeLastContact()
{
    erase(lastContact());
}

std::string ContactList::toString() const
{
    std::stringstream ss;
    ss << "Contact list: " << std::endl;
    for (const auto& contact : m_contacts)
    {
        ss << contact.toString() << std::endl;
    }
    return ss.str();
}

bool ContactList::forceSampleTime(const std::chrono::nanoseconds& dT)
{
    using namespace std::chrono_literals;
    PlannedContact newContact;
    for (int i = 0; i < size(); i++)
    {
        newContact = (*this)[i];

        // if the activation and deactivation time are already a multiple of dT, we skip the
        // contact.
        if (newContact.activationTime % dT == 0ns
            && (newContact.deactivationTime % dT == 0ns
                || newContact.deactivationTime == std::chrono::nanoseconds::max()))
        {
            continue;
        }

        // modify the activation and deactivation time to be a multiple of dT
        // for the activation time we round down, for the deactivation time we round up
        // This will increase the contact duration, but it will not change the contact sequence
        // and location.
        newContact.activationTime -= (newContact.activationTime % dT);

        // if the deactivation time is equal to std::chrono::nanoseconds::max() we do not round
        if (newContact.deactivationTime != std::chrono::nanoseconds::max())
        {
            newContact.deactivationTime += (dT - (newContact.deactivationTime % dT));
        }

        if (!this->editContact(std::next(this->begin(), i), newContact))
        {
            log()->error("[ContactList::forceSampleTime] Failed to edit contact {}. Original "
                         "contact: {}. New contact: {}.",
                         i,
                         (*this)[i].toString(),
                         newContact.toString());
            return false;
        }
    }

    return true;
}

bool ContactList::areContactsSampled(const std::chrono::nanoseconds& dT) const
{
    using namespace std::chrono_literals;
    for (const auto& contact : m_contacts)
    {
        if (contact.activationTime % dT != 0ns
            || (contact.deactivationTime % dT != 0ns
                && contact.deactivationTime != std::chrono::nanoseconds::max()))
        {
            return false;
        }
    }
    return true;
}
