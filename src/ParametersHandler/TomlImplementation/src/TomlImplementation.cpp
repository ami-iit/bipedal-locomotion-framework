/**
 * @file TomlImplementation.cpp
 * @authors Giulio Romualdi
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <string>

#include <BipedalLocomotion/ParametersHandler/TomlImplementation.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ParametersHandler;

template <>
void TomlImplementation::setParameterPrivate<std::vector<bool>>(const std::string& parameterName,
                                                                const std::vector<bool>& parameter)
{
    // vector<bool> is (usually) specialized explicitly to store each bool in a single bit,
    // The range loop using reference leads to a compilation error.
    // https://stackoverflow.com/questions/34079390/range-for-loops-and-stdvectorbool
    m_container.insert_or_assign(parameterName, toml::array());
    for (bool element : parameter)
    {
        m_container[parameterName].as_array()->push_back(element);
    }
}

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
                                      std::chrono::nanoseconds& parameter) const
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

bool TomlImplementation::getParameter(
    const std::string& parameterName,
    GenericContainer::Vector<std::chrono::nanoseconds>::Ref parameter) const
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
                                      const std::chrono::nanoseconds& parameter)
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

void TomlImplementation::setParameter(
    const std::string& parameterName,
    const GenericContainer::Vector<const std::string>::Ref parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void TomlImplementation::setParameter(const std::string& parameterName,
                                      const std::vector<bool>& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void TomlImplementation::setParameter(
    const std::string& parameterName,
    const GenericContainer::Vector<const std::chrono::nanoseconds>::Ref parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

TomlImplementation::TomlImplementation(const toml::table& container)
{
    this->set(container);
}

void TomlImplementation::set(const toml::table& container)
{
    // clear the content of ParametersHandler
    this->clear();

    for (const auto& [k, v] : container)
    {
        if (v.is_table())
        {
            m_lists.emplace(k, std::make_shared<TomlImplementation>(*v.as_table()));
        } else
        {
            m_container.insert(k, v);
        }
    }
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
    } catch (const std::exception& e)
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
    if (m_lists.find(name) != m_lists.end())
    {
        return m_lists.at(name);
    }

    return std::make_shared<TomlImplementation>();
}

bool TomlImplementation::setGroup(const std::string& name, IParametersHandler::shared_ptr newGroup)
{
    auto downcastedPtr = std::dynamic_pointer_cast<TomlImplementation>(newGroup); // to access
                                                                                  // m_container
    if (downcastedPtr == nullptr)
    {
        log()->debug("[TomlImplementation::setGroup] Unable to downcast the pointer to "
                     "TomlImplementation.");

        return false;
    }

    m_lists[name] = downcastedPtr;
    return true;
}

std::string TomlImplementation::toString() const
{
    std::ostringstream stream;
    stream << m_container << std::endl;
    for (const auto& [key, container] : m_lists)
    {
        stream << "[" << key << "]" << std::endl << container->toString();
    }
    return stream.str();
}

bool TomlImplementation::isEmpty() const
{
    return (m_container.size() == 0);
}

void TomlImplementation::clear()
{
    m_container.clear();
    m_lists.clear();
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
