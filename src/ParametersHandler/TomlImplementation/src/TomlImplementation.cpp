/**
 * @file TomlImplementation.cpp
 * @authors Giulio Romualdi
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <string>

#include <BipedalLocomotion/ParametersHandler/TomlImplementation.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ParametersHandler;

bool TomlImplementation::getParameter(const std::string& parameterName, int& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool TomlImplementation::getParameter(const std::string& parameterName, double& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool TomlImplementation::getParameter(const std::string& parameterName,
                                      std::string& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool TomlImplementation::getParameter(const std::string& parameterName, bool& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool TomlImplementation::getParameter(const std::string& parameterName,
                                      GenericContainer::Vector<int>::Ref parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool TomlImplementation::getParameter(const std::string& parameterName,
                                      GenericContainer::Vector<double>::Ref parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool TomlImplementation::getParameter(const std::string& parameterName,
                                      GenericContainer::Vector<std::string>::Ref parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool TomlImplementation::getParameter(const std::string& parameterName,
                                      std::vector<bool>& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

void TomlImplementation::setParameter(const std::string& parameterName, const int& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void TomlImplementation::setParameter(const std::string& parameterName, const double& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void TomlImplementation::setParameter(const std::string& parameterName, const char* parameter)
{
    return setParameterPrivate(parameterName, std::string(parameter));
}

void TomlImplementation::setParameter(const std::string& parameterName,
                                      const std::string& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void TomlImplementation::setParameter(const std::string& parameterName, const bool& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void TomlImplementation::setParameter(const std::string& parameterName,
                                      const GenericContainer::Vector<const int>::Ref parameter)
{
    return setParameterPrivate(parameterName, parameter);
}
void TomlImplementation::setParameter(const std::string& parameterName,
                                      const GenericContainer::Vector<const double>::Ref parameter)
{
    return setParameterPrivate(parameterName, parameter);
}
void TomlImplementation::setParameter(const std::string& parameterName,
                                      const GenericContainer::Vector<const std::string>::Ref parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void TomlImplementation::setParameter(const std::string& parameterName,
                                      const std::vector<bool>& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

TomlImplementation::TomlImplementation(const toml::table& container)
{
    this->set(container);
}

void TomlImplementation::set(const toml::table& container)
{
    m_container = container;
}

bool TomlImplementation::setFromFile(const std::string& filename)
{
    constexpr auto logPrefix = "[TomlImplementation::setFromFile]";

    // load the configuration file
#if TOML_EXCEPTIONS
    try
    {
        auto config = toml::parse_file(filename);
        this->set(config);
    }
    catch(const std::exception& e)
    {
        log()->debug("{} Unable to parse the file named {}. The following exception has been "
                     "thrown {}.",
                     logPrefix,
                     filename,
                     e.what());
        return false;
    }
#else
    auto config = toml::parse_file(filename);
    if (!config)
    {
        log()->debug("{} Unable to parse the file named {}. The following error has been "
                     "thrown {}.",
                     logPrefix,
                     fileName,
                     config.error());
        return false;
    }
    this->set(config);
#endif

    return true;
}

TomlImplementation::weak_ptr TomlImplementation::getGroup(const std::string& name) const
{
    log()->error("[TomlImplementation::getGroup] This function is not implemented");
    return std::make_shared<TomlImplementation>();
}

bool TomlImplementation::setGroup(const std::string& name, IParametersHandler::shared_ptr newGroup)
{
    log()->error("[TomlImplementation::setGroup] This function is not implemented");
    return false;
}

std::string TomlImplementation::toString() const
{
    std::ostringstream stream;
    stream << m_container;
    return stream.str();
}

bool TomlImplementation::isEmpty() const
{
    return (m_container.size() == 0);
}

void TomlImplementation::clear()
{
    m_container.clear();
}

std::shared_ptr<TomlImplementation> TomlImplementation::clonePrivate() const
{
    auto handler = std::make_shared<TomlImplementation>();

    // copy the content of the parameters stored in the handler.
    handler->m_container = this->m_container;

    return handler;
}

IParametersHandler::shared_ptr TomlImplementation::clone() const
{
    return this->clonePrivate();
}
