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

using namespace BipedalLocomotion::Planners;

void BipedalLocomotion::Planners::ContactPhaseList::createPhases()
{
    m_phases.clear();

    std::map<double, std::unordered_map<std::string, ContactList::const_iterator>> activations, deactivations;

    for (ContactListMap::iterator list = m_contactLists.begin(); list != m_contactLists.end(); ++list)
    {
        const std::string& key = list->first;
        for (ContactList::const_iterator step = list->second.begin(); step != list->second.end(); ++step)
        {
            activations[step->activationTime][key] =  step; //By construction, there is only a step given a key and activationTime
            deactivations[step->deactivationTime][key] =  step;
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

    while ((activations.size() + deactivations.size()) > 1)
    {
        if ((activations.size() == 0) || (deactivations.begin()->first <= activations.begin()->first))
        {
            //Here I need to remove from the current phase the contacts that are going to end
            currentPhase.endTime = deactivations.begin()->first;
            m_phases.push_back(currentPhase);

            currentPhase.beginTime = deactivations.begin()->first;

            for (auto& toBeRemoved : deactivations.begin()->second)
            {
                currentPhase.activeContacts.erase(toBeRemoved.first); //Erase all the contacts which are deactivativated in this instant
            }

            deactivations.erase(deactivations.begin());

            if (activations.size() && (deactivations.begin()->first == activations.begin()->first))
            {
                currentPhase.activeContacts.insert(activations.begin()->second.begin(),
                                                   activations.begin()->second.end()); //Add the new contacts to the list.

                activations.erase(activations.begin());
            }
        }
        else // (activations.begin()->first < deactivations.begin()->first)
        {
            currentPhase.endTime = activations.begin()->first;
            m_phases.push_back(currentPhase);

            currentPhase.beginTime = activations.begin()->first;
            currentPhase.activeContacts.insert(activations.begin()->second.begin(),
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
    return m_phases.cbegin();
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

