/**
 * @file VariablesHandler.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <cstddef>

using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::ParametersHandler;

bool VariablesHandler::VariableDescription::isValid() const
{
    return (offset >= 0) && (size >= 0) && (m_elementsName.size() >= 0)
           && (m_elementsNameMap.size() >= 0);
}

VariablesHandler::VariableDescription VariablesHandler::VariableDescription::InvalidVariable()
{
    VariablesHandler::VariableDescription tmp;
    tmp.offset = tmp.size = InvalidIndex;
    return tmp;
}

std::ptrdiff_t VariablesHandler::VariableDescription::getElementIndex(const std::string& name) const
{
    // find the element index associated to the given name
    auto element = m_elementsNameMap.find(name);

    if (element == m_elementsNameMap.end())
    {
        log()->error("[VariableDescription::getElementIndex] Unable to find the element named: {}. "
                     "an InvalidIndex will be returned.",
                     name);
        return InvalidIndex;
    }

    return element->second + offset;
}

std::ptrdiff_t
VariablesHandler::VariableDescription::getElementIndex(std::ptrdiff_t localIndex) const
{
    if (localIndex >= size)
    {
        log()->error("[VariableDescription::getElementIndex] The localIndex is greather than the "
                     "size of the variable. InvalidIndex will be returned.");

        return InvalidIndex;
    }

    return localIndex + offset;
}

bool VariablesHandler::initialize(std::weak_ptr<const IParametersHandler> handler) noexcept
{
    // clear the content of the handler
    this->clear();

    constexpr auto logPrefix = "[VariablesHandler::initialize]";
    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid ParametersHandler pointer.", logPrefix);
        return false;
    }

    std::vector<std::string> names;
    std::vector<int> sizes;
    if (!ptr->getParameter("variables_name", names))
    {
        log()->error("{} Unable to get the name of the variables.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("variables_size", sizes))
    {
        log()->error("{} Unable to get the size of the variables.", logPrefix);
        return false;
    }

    if (names.size() != sizes.size())
    {
        log()->error("{} The size of the list containing the names is different from the size of "
                     "the list containing the sizes. Size of variables_name: {}. Size of "
                     "variables_size: {}",
                     logPrefix,
                     names.size(),
                     sizes.size());
        return false;
    }

    // the size must be a strictly positive number
    auto iterator = std::find_if(sizes.begin(), sizes.end(), [](int size) { return size <= 0; });
    if (iterator != sizes.end())
    {
        log()->error("{} There exist a non positive element in the variables_size list. The size "
                     "must be a strictly positive number.",
                     logPrefix);
        return false;
    }

    std::vector<std::string> elementsNameVector;
    for (int i = 0; i < names.size(); i++)
    {
        // check if the elements name vector has been provided
        if (ptr->getParameter(names[i] + "_elements_name", elementsNameVector))
        {

            if (!this->addVariable(names[i], sizes[i], elementsNameVector))
            {
                log()->error("{} Unable to add the variable named {} having a size equal to {}.",
                             logPrefix,
                             names[i],
                             sizes[i]);
                return false;
            }
        } else
        {
            log()->info("{} The parameter {}_elements_name is not found. The default one is used",
                        logPrefix,
                        names[i]);

            if (!this->addVariable(names[i], sizes[i]))
            {
                log()->error("{} Unable to add the variable named {} having a size equal to {}.",
                             logPrefix,
                             names[i],
                             sizes[i]);
                return false;
            }
        }
    }

    return true;
}

bool VariablesHandler::addVariable(const std::string& name, const std::size_t& size) noexcept
{
    std::vector<std::string> elementsName(size);
    for (int i = 0; i < size; i++)
    {
        elementsName[i] = name + "_" + std::to_string(i);
    }

    return this->addVariable(name, elementsName.size(), elementsName);

    return true;
}

bool VariablesHandler::addVariable(const std::string& name,
                                   const std::vector<std::string>& elementsName) noexcept
{
    return this->addVariable(name, elementsName.size(), elementsName);
}

bool VariablesHandler::addVariable(const std::string& name,
                                   const std::size_t& size,
                                   const std::vector<std::string>& elementsName) noexcept
{
    // if the variable already exist cannot be added again.
    if (m_variables.find(name) != m_variables.end())
    {
        log()->error("[VariableHandler::addVariable] The variable named {} already exists.", name);
        return false;
    }

    if (elementsName.size() != size)
    {
        log()->error("[VariableHandler::addVariable] The size of the vector of the element is "
                     "different from the expected one. Expected: {}, Retrieved {}.",
                     size,
                     elementsName.size());
        return false;
    }

    VariablesHandler::VariableDescription description;
    description.size = size;
    description.offset = m_numberOfVariables;
    description.name = name;
    description.m_elementsName = elementsName;
    for (int i = 0; i < elementsName.size(); i++)
    {
        const auto& elementName = elementsName[i];
        auto outcome = description.m_elementsNameMap.insert({elementName, i});
        if (!outcome.second)
        {
            log()->error("[VariableHandler::addVariable] Unable to add the element {} in the "
                         "variable {}. The element already exists.",
                         elementName,
                         name);
            return false;
        }
    }

    m_variables.emplace(name, description);
    m_numberOfVariables += size;

    return true;
}

const VariablesHandler::VariableDescription&
VariablesHandler::getVariable(const std::string& name) const noexcept
{
    auto variable = m_variables.find(name);

    // if the variable is present its IndexRange is returned otherwise an
    // invalid IndexRange is provided to the user

    if (variable != m_variables.end())
        return variable->second;
    else
        return m_invalidVariable;
}

bool VariablesHandler::getVariable(const std::string& name, //
                                   VariablesHandler::VariableDescription& description) const noexcept
{
    return (description = this->getVariable(name)).isValid();
}

const std::size_t& VariablesHandler::getNumberOfVariables() const noexcept
{
    return m_numberOfVariables;
}

std::string VariablesHandler::toString() const noexcept
{
    std::string out;
    for (const auto& [key, variable] : m_variables)
    {
        out += key + " size: " + std::to_string(variable.size)
               + ", offset: " + std::to_string(variable.offset) + " elements name:";
        for (const auto& name : variable.m_elementsName)
        {
            out += " " + name;
        }
    }

    return out;
}

void VariablesHandler::clear() noexcept
{
    m_numberOfVariables = 0;
    m_variables.clear();
}
