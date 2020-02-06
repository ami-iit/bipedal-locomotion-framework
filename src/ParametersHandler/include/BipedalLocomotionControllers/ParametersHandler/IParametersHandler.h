/**
 * @file IParametersHandler.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_PARAMETERS_HANDLER_IPARAMETERS_HANDLER_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_PARAMETERS_HANDLER_IPARAMETERS_HANDLER_H

#include <memory>
#include <string>

namespace BipedalLocomotionControllers
{
namespace ParametersHandler
{

/**
 * Parameters handler interface.
 * @tparam Derived type of the Derived class. Necessary to implement the Curiously recurring
 * template pattern
 */
template <class Derived> class IParametersHandler
{
public:
    /**
     * Get a parameter from the handler.
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @tparam T type of the parameter
     * @return true/false in case of success/failure
     */
    template <typename T> bool getParameter(const std::string& parameterName, T& parameter) const;

    /**
     * Get a Group from the handler.
     * @param name name of the group
     * @return A pointer to IParametersHandler, If the group is not found the pointer is equal to
     * nullptr
     */
    std::unique_ptr<IParametersHandler<Derived>> getGroup(const std::string& name) const;
};
} // namespace ParametersHandler
} // namespace BipedalLocomotionControllers

#include "IParametersHandler.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_PARAMETERS_HANDLER_IPARAMETERS_HANDLER_H
