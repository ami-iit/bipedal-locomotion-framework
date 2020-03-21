/**
 * @file YarpImplementation.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <yarp/os/Bottle.h>

#include <cassert>
#include <string>

#include <BipedalLocomotionControllers/ParametersHandler/YarpImplementation.h>

using namespace BipedalLocomotionControllers::ParametersHandler;

bool YarpImplementation::getParameter(const std::string& parameterName, int& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool YarpImplementation::getParameter(const std::string& parameterName, double& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool YarpImplementation::getParameter(const std::string& parameterName,
                                      std::string& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool YarpImplementation::getParameter(const std::string& parameterName, bool& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool YarpImplementation::getParameter(const std::string& parameterName,
                                      GenericContainer::Vector<int>& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool YarpImplementation::getParameter(const std::string& parameterName,
                                      GenericContainer::Vector<double>& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool YarpImplementation::getParameter(const std::string& parameterName,
                                      GenericContainer::Vector<std::string>& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool YarpImplementation::getParameter(const std::string& parameterName,
                                      std::vector<bool>& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

void YarpImplementation::setParameter(const std::string& parameterName, const int& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void YarpImplementation::setParameter(const std::string& parameterName, const double& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void YarpImplementation::setParameter(const std::string& parameterName, const char* parameter)
{
    return setParameterPrivate(parameterName, std::string(parameter));
}

void YarpImplementation::setParameter(const std::string& parameterName,
                                      const std::string& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void YarpImplementation::setParameter(const std::string& parameterName, const bool& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void YarpImplementation::setParameter(const std::string& parameterName,
                                      const GenericContainer::Vector<const int>& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}
void YarpImplementation::setParameter(const std::string& parameterName,
                                      const GenericContainer::Vector<const double>& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}
void YarpImplementation::setParameter(const std::string& parameterName,
                                      const GenericContainer::Vector<const std::string>& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void YarpImplementation::setParameter(const std::string& parameterName,
                                      const std::vector<bool>& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

YarpImplementation::YarpImplementation(const yarp::os::Searchable& searchable)
{
    set(searchable);
}

void YarpImplementation::set(const yarp::os::Searchable& searchable)
{
    clear();

    yarp::os::Bottle bot;
    bot.fromString(searchable.toString());

    for (size_t i = 0; i < bot.size(); i++) // all sublists are included in a new object
    {
        yarp::os::Value& bb = bot.get(i);

        yarp::os::Bottle* sub = bb.asList();
        if ((sub) && (sub->size() > 1))
        {
            std::string name = sub->get(0).toString();
            yarp::os::Bottle* subSub = sub->get(1).asList();
            if ((subSub) && (subSub->size() > 1))
            {
                m_lists.emplace(name, std::make_shared<YarpImplementation>(*sub));
            } else
            {
                m_container.add(bb);
            }

        } else
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

    return std::make_shared<YarpImplementation>();
}

bool YarpImplementation::setGroup(const std::string& name, IParametersHandler::shared_ptr newGroup)
{
    auto downcastedPtr = std::dynamic_pointer_cast<YarpImplementation>(newGroup); // to access
                                                                                  // m_container
    if (downcastedPtr == nullptr)
    {
        std::cerr << "[YarpImplementation::setGroup] Unable to downcast the pointer to "
                     "YarpImplementation."
                  << std::endl;
        return false;
    }

    yarp::os::Bottle backup = downcastedPtr->m_container;
    yarp::os::Bottle nameAdded;
    nameAdded.add(yarp::os::Value(name));
    nameAdded.append(backup);
    downcastedPtr->m_container = nameAdded; // This is all to add the name at the beginning of the
                                            // bottle
    m_lists[name] = newGroup;

    return true;
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
    // We check if the container is null and there are no lists.
    // A special case is when the container has a single string element.
    // This is the case for newly created groups, where the container has only the name of the group
    // itself.
    return ((m_container.size() == 0 || (m_container.size() == 1 && m_container.get(0).isString()))
            && (m_lists.size() == 0));
}

void YarpImplementation::clear()
{
    m_container.clear();
    m_lists.clear();
}
