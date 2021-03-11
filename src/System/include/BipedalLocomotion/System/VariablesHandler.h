/**
 * @file VariablesHandler.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <string>
#include <unordered_map>

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_VARIABLES_HANDLER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_VARIABLES_HANDLER_H

namespace BipedalLocomotion
{
namespace System
{

/**
 * VariableHandler is useful to handle variables in an optimization problem, Their name, dimension
 * and position
 */
class VariablesHandler
{
public:
    struct VariableDescription
    {
        std::ptrdiff_t offset;
        std::ptrdiff_t size;
        std::string name;

        bool isValid() const;
        static VariableDescription InvalidVariable();
    };

private:
    std::unordered_map<std::string, VariableDescription> m_variables; /**< Map containing the name
                                                                         of a variable and its
                                                                         index range */
    std::size_t m_numberOfVariables{0}; /**< Total number of Variable seen as scalar */

public:
    /**
     * Add a new variable to the list
     * @param name of the variable
     * @param size the size of the variable
     * @return true/false in case of success/failure
     */
    bool addVariable(const std::string& name, const std::size_t& size) noexcept;

    /**
     * Get a variable from the list
     * @param name of the variable
     * @return the index range associated to the variable
     */
    VariableDescription getVariable(const std::string& name) const noexcept;

    /**
     * Get a variable from the list
     * @param name of the variable
     * @param[out] description the description of the variable
     * @return true/false in case of success/failure
     */
    bool getVariable(const std::string& name, VariableDescription& description) const noexcept;

    /**
     * Get the number of variables
     * @return the total number of variables
     */
    const std::size_t& getNumberOfVariables() const noexcept;
};
} // namespace System
} // namespace BipedalLocomotion
#endif // BIPEDAL_LOCOMOTION_SYSTEM_VARIABLES_HANDLER_H
