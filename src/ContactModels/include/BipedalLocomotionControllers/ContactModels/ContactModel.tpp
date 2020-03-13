/**
 * @file ContactModel.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_CONTACT_MODELS_CONTACT_MODEL_TPP
#define BIPEDAL_LOCOMOTION_CONTROLLERS_CONTACT_MODELS_CONTACT_MODEL_TPP

#include <iostream>

#include <BipedalLocomotionControllers/ContactModels/ContactModel.h>

namespace BipedalLocomotionControllers
{
namespace ContactModels
{
template <class T>
bool ContactModel::getVariable(const std::unordered_map<std::string, std::any>& variables,
                               const std::string& variableName,
                               T& variable)
{
    const auto& var = variables.find(variableName);

    if (var == variables.end())
    {
        std::cerr << "[ContactModel::getVariable] The variables named " << variableName
                  << " cannot be found." << std::endl;
        return false;
    }

    try
    {
        variable = std::any_cast<T>(var->second);
    } catch (const std::bad_any_cast& e)
    {
        std::cerr << "[ContactModel::getVariable] The type of the variable is not the one expected."
                  << std::endl;
        return false;
    }

    return true;
}

} // namespace ContactModels
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_CONTACT_MODELS_CONTACT_MODEL_TPP
