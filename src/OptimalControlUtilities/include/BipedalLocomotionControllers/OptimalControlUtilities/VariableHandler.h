/**
 * @file VariableHandler.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <string>
#include <unordered_map>

#include <iDynTree/Core/Utils.h>

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_VARIABLE_HANDLER_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_VARIABLE_HANDLER_H

namespace BipedalLocomotionControllers
{
/**
 * VariableHandler is useful to handle variables in an optimization problem, Their name, dimension
 * and position
 */
class VariableHandler
{

    std::unordered_map<std::string, iDynTree::IndexRange> m_variables; /**< Map containing the name
                                                                          of a variable and its
                                                                          index range */
    int m_numberOfVariables{0}; /**< Total number of Variable seen as scalar */

public:
    /**
     * Add a mew variable to the list
     * @param name of the variable
     * @param size the size of the variable
     * @return true/false in case of success/failure
     */
    bool addVariable(const std::string& name, const unsigned int& size) noexcept;

    /**
     * Add a mew variable to the list
     * @param name of the variable
     * @return the index range associated to the variable
     */
    iDynTree::IndexRange getVariable(const std::string& name) const noexcept;

    /**
     * Get the number of variables
     * @return the total number of variables
     */
    int getNumberOfVariables() const noexcept;
};
} // namespace BipedalLocomotionControllers
#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_VARIABLE_HANDLER_H
