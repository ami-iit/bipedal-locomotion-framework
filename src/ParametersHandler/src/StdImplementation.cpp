/**
 * @file StdImplementation.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <string>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ParametersHandler;

void StdImplementation::setParameter(const std::string& parameterName,
                                     const GenericContainer::Vector<const int>::Ref parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void StdImplementation::setParameter(const std::string& parameterName,
                                     const GenericContainer::Vector<const double>::Ref parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void StdImplementation::setParameter(const std::string& parameterName,
                                     const GenericContainer::Vector<const std::string>::Ref parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

bool StdImplementation::getParameter(const std::string& parameterName,
                                     GenericContainer::Vector<int>::Ref parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool StdImplementation::getParameter(const std::string& parameterName,
                                     GenericContainer::Vector<double>::Ref parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool StdImplementation::getParameter(const std::string& parameterName,
                                     GenericContainer::Vector<std::string>::Ref parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool StdImplementation::getParameter(const std::string& parameterName, int& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool StdImplementation::getParameter(const std::string& parameterName,
                                     double& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool StdImplementation::getParameter(const std::string& parameterName,
                                     std::string& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool StdImplementation::getParameter(const std::string& parameterName, bool& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool StdImplementation::getParameter(const std::string& parameterName,
                                     std::vector<bool>& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

void StdImplementation::setParameter(const std::string& parameterName, const int& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void StdImplementation::setParameter(const std::string& parameterName,
                                     const double& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void StdImplementation::setParameter(const std::string& parameterName, const char* parameter)
{
    return setParameterPrivate(parameterName, std::string(parameter));
}

void StdImplementation::setParameter(const std::string& parameterName,
                                     const std::string& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void StdImplementation::setParameter(const std::string& parameterName, const bool& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void StdImplementation::setParameter(const std::string& parameterName,
                                     const std::vector<bool>& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

bool StdImplementation::setGroup(const std::string& name, IParametersHandler::shared_ptr newGroup)
{
    auto downcastedPtr = std::dynamic_pointer_cast<StdImplementation>(newGroup);
    if (downcastedPtr == nullptr)
    {
        BipedalLocomotion::log()->error("[StdImplementation::setGroup] Unable to downcast the "
                                        "pointer to StdImplementation.");
        return false;
    }

    m_map[name] = std::make_any<shared_ptr>(downcastedPtr);

    return true;
}

IParametersHandler::weak_ptr StdImplementation::getGroup(const std::string& name) const
{
    auto group = m_map.find(name);
    if (group == m_map.end())
        return std::make_shared<StdImplementation>();

    shared_ptr map;
    try
    {
        map = std::any_cast<shared_ptr>(group->second);
    } catch (const std::bad_any_cast& exception)
    {
        log()->warn("[StdImplementation::getGroup] The element named '{}' is not a "
                    "'std::unordered_map<std::string, std::any>'",
                    name);

        return std::make_shared<StdImplementation>();
    }

    return map;
}

void StdImplementation::set(const std::unordered_map<std::string, std::any>& object)
{
    m_map = object;
}

std::string StdImplementation::toString() const
{
    std::string key;
    for (const auto& parameters : m_map)
        key += parameters.first + " ";

    return key;
}

bool StdImplementation::isEmpty() const
{
    return m_map.size() == 0;
}

void StdImplementation::clear()
{
    m_map.clear();
}

IParametersHandler::shared_ptr StdImplementation::clone() const
{
    auto handler = std::make_shared<StdImplementation>();

    // copy the content of the parameters stored in the handler.
    handler->m_map = this->m_map;

    return handler;
}
