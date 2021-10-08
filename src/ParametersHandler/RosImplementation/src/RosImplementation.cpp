/**
 * @file RosImplementation.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <cassert>
#include <string>

#include <BipedalLocomotion/ParametersHandler/RosImplementation.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ParametersHandler;

bool RosImplementation::getParameter(const std::string& parameterName, int& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool RosImplementation::getParameter(const std::string& parameterName, double& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool RosImplementation::getParameter(const std::string& parameterName,
                                      std::string& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool RosImplementation::getParameter(const std::string& parameterName, bool& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool RosImplementation::getParameter(const std::string& parameterName,
                                      GenericContainer::Vector<int>::Ref parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool RosImplementation::getParameter(const std::string& parameterName,
                                      GenericContainer::Vector<double>::Ref parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool RosImplementation::getParameter(const std::string& parameterName,
                                      GenericContainer::Vector<std::string>::Ref parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool RosImplementation::getParameter(const std::string& parameterName,
                                      std::vector<bool>& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

void RosImplementation::setParameter(const std::string& parameterName, const int& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void RosImplementation::setParameter(const std::string& parameterName, const double& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void RosImplementation::setParameter(const std::string& parameterName, const char* parameter)
{
    return setParameterPrivate(parameterName, std::string(parameter));
}

void RosImplementation::setParameter(const std::string& parameterName,
                                      const std::string& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void RosImplementation::setParameter(const std::string& parameterName, const bool& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void RosImplementation::setParameter(const std::string& parameterName,
                                      const GenericContainer::Vector<const int>::Ref parameter)
{
    return setParameterPrivate(parameterName, parameter);
}
void RosImplementation::setParameter(const std::string& parameterName,
                                      const GenericContainer::Vector<const double>::Ref parameter)
{
    return setParameterPrivate(parameterName, parameter);
}
void RosImplementation::setParameter(const std::string& parameterName,
                                      const GenericContainer::Vector<const std::string>::Ref parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void RosImplementation::setParameter(const std::string& parameterName,
                                      const std::vector<bool>& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

RosImplementation::RosImplementation(const ros::NodeHandle& handle)
{
    set(handle);
}

void RosImplementation::set(const ros::NodeHandle& handle)
{

    auto nthOccurrence = [](const std::string& str, const std::string& findMe, int nth) {
        size_t pos = -1;
        int cnt = 0;

        while (cnt != nth)
        {
            pos += 1;
            pos = str.find(findMe, pos);
            if (pos == std::string::npos)
                return std::string::npos;
            cnt++;
        }
        return pos;
    };

    // reset the content of the internal variables
    m_lists.clear();
    m_handle = handle;
    const auto namespaceWithSlash = m_handle.getNamespace() == "/" //
                                        ? "/"
                                        : m_handle.getNamespace() + "/";
    const int slashOccurence = m_handle.getNamespace() == "/" ? 2 : 3;
    const int temp = m_handle.getNamespace() == "/" ? 0 : 1;

    std::vector<std::string> allParameters;
    handle.getParamNames(allParameters);
    std::string tempSubstring;
    for (const auto& param : allParameters)
    {
        if (param.rfind(namespaceWithSlash, 0) == 0)
        { // pos=0 limits the search to the prefix
            // "/root/group/param"
            const auto s = param.substr(namespaceWithSlash.size());
            const auto slashPosition = nthOccurrence(s, "/", 1);
            if (slashPosition != std::string::npos)
            {
                tempSubstring = s.substr(0, slashPosition);

                // check if the group has been already added
                if (m_lists.find(tempSubstring) == m_lists.end())
                {
                    // create a temporary handle
                    ros::NodeHandle tempHandle(handle, tempSubstring);
                    m_lists.emplace(tempSubstring,
                                    std::make_shared<RosImplementation>(tempHandle));
                }
            }
        }
    }
}

RosImplementation::weak_ptr RosImplementation::getGroup(const std::string& name) const
{
    if (m_lists.find(name) != m_lists.end())
    {
        return m_lists.at(name);
    }

    return std::make_shared<RosImplementation>();
}

bool RosImplementation::setGroup(const std::string& name, IParametersHandler::shared_ptr newGroup)
{
    log()->debug("[RosImplementation::setGroup] This function is currently not implemented.");
    return false;
}

std::string RosImplementation::toString() const
{
    std::string output = "";

    const auto namespaceWithSlash = m_handle.getNamespace() == "/" //
                                        ? "/"
                                        : m_handle.getNamespace() + "/";

    std::vector<std::string> allParameters;
    m_handle.getParamNames(allParameters);
    for (const auto& param : allParameters)
    {
        if (param.rfind(namespaceWithSlash, 0) == 0)
        {
            output += param + " ";
        }
    }

    return output;
}

bool RosImplementation::isEmpty() const
{
    const auto namespaceWithSlash = m_handle.getNamespace() == "/" //
                                        ? "/"
                                        : m_handle.getNamespace() + "/";

    std::vector<std::string> allParameters;
    m_handle.getParamNames(allParameters);
    for (const auto& param : allParameters)
    {
        if (param.rfind(namespaceWithSlash, 0) == 0)
        {
            return false;
        }
    }

    return true;
}

void RosImplementation::clear()
{
    // get all the parameters
    const auto namespaceWithSlash = m_handle.getNamespace() == "/" //
                                        ? "/"
                                        : m_handle.getNamespace() + "/";

    std::vector<std::string> allParameters;
    m_handle.getParamNames(allParameters);
    for (const auto& param : allParameters)
    {
        if (param.rfind(namespaceWithSlash, 0) == 0)
        {
            m_handle.deleteParam(param);
        }
    }
}

IParametersHandler::shared_ptr RosImplementation::clone() const
{
    auto handler = std::make_shared<RosImplementation>();
    handler->m_handle = this->m_handle;
    handler->m_lists = this->m_lists;

    return handler;
}
