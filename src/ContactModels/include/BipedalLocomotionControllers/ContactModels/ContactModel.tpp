/**
 * @file ContactModel.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iostream>

namespace BipedalLocomotionControllers
{
namespace ContactModels
{
template <class T>
bool ContactModel::getVariable(const std::unordered_map<std::string, std::any>& variables,
                               const std::string& variableName, T& variable)
{
    const auto& var = variables.find(variableName);

    if (var == variables.end())
    {
        std::cerr << "[ContactModel::getVariable] The variables named " << variableName
                  << " cannot be found." << std::endl;
        return false;
    }

    variable = std::any_cast<T>(var->second);

    return true;
}

} // namespace ContactModels
} // namespace BipedalLocomotionControllers
