/**
 * @file VariablesHandler.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iostream>
#include <BipedalLocomotion/System/VariablesHandler.h>

using namespace BipedalLocomotion::System;

bool VariablesHandler::VariableDescription::isValid() const
{
    return (offset >= 0) && (size >= 0);
}

VariablesHandler::VariableDescription VariablesHandler::VariableDescription::InvalidVariable()
{
    VariablesHandler::VariableDescription tmp;
    tmp.offset = tmp.size = -1;
    return tmp;
}

bool VariablesHandler::addVariable(const std::string& name, const std::size_t& size) noexcept
{
    // if the variable already exist cannot be added again.
    if (m_variables.find(name) != m_variables.end())
    {
        std::cerr << "[VariableHandler::addVariable] The variable name " << name
                  << " already exists";
        return false;
    }

    VariablesHandler::VariableDescription description;
    description.size = size;
    description.offset = m_numberOfVariables;
    description.name = name;

    m_variables.emplace(name, description);
    m_numberOfVariables += size;

    return true;
}

VariablesHandler::VariableDescription VariablesHandler::getVariable(const std::string& name) const
    noexcept
{
    auto variable = m_variables.find(name);

    // if the variable is present its IndexRange is returned otherwise an
    // invalid IndexRange is provided to the user

    if (variable != m_variables.end())
        return variable->second;
    else
        return VariablesHandler::VariableDescription::InvalidVariable();
}

bool VariablesHandler::getVariable(const std::string& name,
                                   VariablesHandler::VariableDescription& description) const
    noexcept
{
    return (description = this->getVariable(name)).isValid();
}

const std::size_t& VariablesHandler::getNumberOfVariables() const noexcept
{
    return m_numberOfVariables;
}
