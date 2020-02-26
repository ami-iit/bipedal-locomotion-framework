/**
 * @file YarpImplementation.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/ParametersHandler/YarpImplementation.h>
#include <yarp/os/Bottle.h>

#include <string>
#include <cassert>

using namespace BipedalLocomotionControllers::ParametersHandler;

YarpImplementation::YarpImplementation(const yarp::os::Searchable& searchable)
{
    set(searchable);
}

void YarpImplementation::set(const yarp::os::Searchable &searchable)
{
    m_container.clear();
    m_lists.clear();

    yarp::os::Bottle bot;
    bot.fromString(searchable.toString());

    for (size_t i = 0; i < bot.size(); i++) { //all sublists are included in a new object
        yarp::os::Value& bb = bot.get(i);

        yarp::os::Bottle* sub = bb.asList();
        if ((sub) && (sub->size() > 1)) {
            std::string name = sub->get(0).toString();
            yarp::os::Bottle* subSub = sub->get(1).asList();
            if ((subSub) && (subSub->size() > 1))
            {
                m_lists.emplace(name, make_shared(*subSub));
            }
            else
            {
                m_container.add(bb);
            }

        }
        else
        {
            m_container.add(bb);
        }
    }
}

YarpImplementation::weak_ptr YarpImplementation::getGroup(const std::string& name) const
{
    if (m_lists.find(name) != m_lists.end())
    {
        return m_lists.at(name);
    }

    return YarpImplementation::make_shared();
}

void YarpImplementation::setGroup(const std::string &name, IParametersHandler<YarpImplementation>::shared_ptr newGroup)
{
    auto downcastedPtr = std::dynamic_pointer_cast<YarpImplementation>(newGroup); //to access m_container
    assert(downcastedPtr);
    yarp::os::Bottle backup = downcastedPtr->m_container;
    yarp::os::Bottle nameAdded;
    nameAdded.add(yarp::os::Value(name));
    nameAdded.append(backup);
    downcastedPtr->m_container = nameAdded; //This is all to add the name at the beginning of the bottle
    m_lists[name] = newGroup;
}

std::string YarpImplementation::toString() const
{
    std::string output = m_container.toString();
    for (auto& group : m_lists)
    {
        output += " (" + group.second->toString() + ")";
    }
    return output;
}

bool YarpImplementation::isEmpty() const
{
    // if the toString method returns an empty string means that the handler is empty
    return ((m_container.size() == 0 || (m_container.size() == 1 && m_container.get(0).isString())) && (m_lists.size() == 0));
}
