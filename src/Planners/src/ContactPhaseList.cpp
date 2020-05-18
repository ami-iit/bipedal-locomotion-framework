/**
 * @file ContactPhase.cpp
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Planners/ContactPhaseList.h>

#include <iostream>
#include <map>
#include <cassert>
#include <algorithm>

using namespace BipedalLocomotion::Planners;

void BipedalLocomotion::Planners::ContactPhaseList::createPhases()
{
    m_phases.clear();

    std::map<double, std::vector<ContactReference>> activations, deactivations;

    for (ContactListMap::iterator list = m_contactLists.begin(); list != m_contactLists.end(); ++list)
    {
        const std::string& key = list->first;
        for (ContactList::const_iterator step = list->second.begin(); step != list->second.end(); ++step)
        {
            ContactReference newReference;
            newReference.listLabel = key;
            newReference.contact_it = step;

            activations[step->activationTime].push_back(newReference);
            deactivations[step->deactivationTime].push_back(newReference);
        }
    }

    if (!activations.size())
    {
        return;
    }

    ContactPhase currentPhase;
    currentPhase.beginTime = activations.begin()->first;
    currentPhase.activeContacts = activations.begin()->second;

    activations.erase(activations.begin());

    while (activations.size())
    {
        if (deactivations.begin()->first <= activations.begin()->first)
        {
            //Here I need to remove from the current phase the contacts that are going to end
            currentPhase.endTime = deactivations.begin()->first;
            m_phases.push_back(currentPhase);

            currentPhase.beginTime = deactivations.begin()->first;
            const std::vector<ContactReference>& toBeRemoved = deactivations.begin()->second;

            //The following lambda returns true if the label inside reference is contained in toBeRemoved
            auto deleteLambda = [toBeRemoved](const ContactReference &reference)
            {
                const std::string& key = reference.listLabel;
                std::vector<ContactReference>::const_iterator it = std::find_if(toBeRemoved.begin(), toBeRemoved.end(),
                                                                                [key](const ContactReference& ref){return ref.listLabel == key;});

                return  it != toBeRemoved.end();
            };

            //Remove from activeContacts all the elements for which deleteLambda returns true.
            currentPhase.activeContacts.erase(std::remove_if(currentPhase.activeContacts.begin(),
                                                             currentPhase.activeContacts.end(),
                                                             deleteLambda), currentPhase.activeContacts.end());

            deactivations.erase(deactivations.begin());

            if (deactivations.begin()->first == activations.begin()->first)
            {
                currentPhase.activeContacts.insert(currentPhase.activeContacts.end(),
                                                   activations.begin()->second.begin(),
                                                   activations.begin()->second.end()); //Add the new contacts to the list.

                activations.erase(activations.begin());
            }
        }
        else // (activations.begin()->first < deactivations.begin()->first)
        {
            currentPhase.endTime = activations.begin()->first;
            m_phases.push_back(currentPhase);

            currentPhase.beginTime = activations.begin()->first;
            currentPhase.activeContacts.insert(currentPhase.activeContacts.end(),
                                               activations.begin()->second.begin(),
                                               activations.begin()->second.end()); //Add the new contacts to the list.

            activations.erase(activations.begin());
        }
    }

    assert(deactivations.size() == 1); //Only one element should be left (deactivations and activations were equal in number, but the head of activations was deleted at the beginning).
    currentPhase.endTime = deactivations.begin()->first;
    m_phases.push_back(currentPhase);
}

void ContactPhaseList::setLists(const ContactListMap &contactLists)
{
    m_contactLists = contactLists;
    createPhases();
}

bool ContactPhaseList::setLists(const std::initializer_list<ContactList> &contactLists)
{
    m_contactLists.clear();
    for (const ContactList& list : contactLists)
    {
        std::pair<ContactListMap::iterator, bool> res = m_contactLists.insert(ContactListMap::value_type(list.defaultName(), list));

        if (!res.second)
        {
            std::cerr << "[ContactPhaseList::setLists] Multiple items have the same defaultName." <<std::endl;
            return false;
        }
    }

    createPhases();

    return true;
}

const BipedalLocomotion::Planners::ContactListMap &BipedalLocomotion::Planners::ContactPhaseList::lists() const
{
    return m_contactLists;
}

ContactPhaseList::const_iterator ContactPhaseList::begin() const
{
    return m_phases.begin();
}

ContactPhaseList::const_iterator ContactPhaseList::cbegin() const
{
    return m_phases.end();
}

ContactPhaseList::const_reverse_iterator ContactPhaseList::rbegin() const
{
    return m_phases.rbegin();
}

ContactPhaseList::const_reverse_iterator ContactPhaseList::crbegin() const
{
    return m_phases.crbegin();
}

ContactPhaseList::const_iterator ContactPhaseList::end() const
{
    return m_phases.end();
}

ContactPhaseList::const_iterator ContactPhaseList::cend() const
{
    return m_phases.cend();
}

ContactPhaseList::const_reverse_iterator ContactPhaseList::rend() const
{
    return m_phases.rend();
}

ContactPhaseList::const_reverse_iterator ContactPhaseList::crend() const
{
    return m_phases.crend();
}

const ContactPhase &ContactPhaseList::operator[](size_t index) const
{
    return m_phases[index];
}

ContactPhaseList::const_iterator ContactPhaseList::firstPhase() const
{
    return m_phases.begin();
}

ContactPhaseList::const_iterator ContactPhaseList::lastPhase() const
{
    return --m_phases.end();
}

size_t ContactPhaseList::size() const
{
    return m_phases.size();
}

void ContactPhaseList::clear()
{
    m_phases.clear();
    m_contactLists.clear();
}

