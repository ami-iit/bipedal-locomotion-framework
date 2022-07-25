/**
 * @file VariablesHandler.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_VARIABLES_HANDLER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_VARIABLES_HANDLER_H

#include <cstddef>
#include <string>
#include <memory>
#include <unordered_map>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace System
{

class VariablesHandler;

/**
 * VariableHandler is useful to handle variables in an optimization problem, Their name, dimension
 * and position
 */
class VariablesHandler
{
public:
    class VariableDescription
    {
    public:
        static constexpr std::ptrdiff_t InvalidIndex{-1};
        std::ptrdiff_t offset{InvalidIndex};
        std::ptrdiff_t size{InvalidIndex};
        std::string name;

        bool isValid() const;

        std::ptrdiff_t getElementIndex(const std::string& name) const;
        std::ptrdiff_t getElementIndex(std::ptrdiff_t localIndex) const;

        static VariableDescription InvalidVariable();

    private:
        std::unordered_map<std::string, std::ptrdiff_t> m_elementsNameMap;
        std::vector<std::string> m_elementsName;

        friend class VariablesHandler;
    };

private:
    std::unordered_map<std::string, VariableDescription> m_variables; /**< Map containing the name
                                                                         of a variable and its
                                                                         index range */
    std::size_t m_numberOfVariables{0}; /**< Total number of Variable seen as scalar */

    /**
     * An invalid variable
     */
    VariableDescription m_invalidVariable{VariablesHandler::VariableDescription::InvalidVariable()};

public:
    /**
     * Initialize the VariablesHandler class. This method calls VariablesHandler::addVariable using
     * the parameters stored in the handler
     * @param handler pointer to parameters handler
     * @notice The following parameters are required
     * |  Parameter Name  |       Type       |                                       Description                                      | Mandatory |
     * |:----------------:|:----------------:|:--------------------------------------------------------------------------------------:|:---------:|
     * | `variables_name` | `vector<string>` |                        List containing the name of the variables                       |    Yes    |
     * | `variables_size` |   `vector<int>`  | List containing the size of the variables. The size must be a strictly positive number |    Yes    |
     * | `<variable_name>_elements_name` |   `vector<string>`  |  List containing the name of the elements associated to a variable.  |    Yes    |
     * @warning The previous content of the VariablesHandler is erased.
     * @return true/false in case of success/failure
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) noexcept;

    /**
     * Add a new variable to the list
     * @param name of the variable
     * @param size the size of the variable
     * @return true/false in case of success/failure
     */
    bool addVariable(const std::string& name, const std::size_t& size) noexcept;

    /**
     * Add a new variable to the list
     * @param name of the variable
     * @param size the size of the variable
     * @param elementsName vector containing the name associated to each elements of the variable
     * @return true/false in case of success/failure
     */
    bool addVariable(const std::string& name,
                     const std::size_t& size,
                     const std::vector<std::string>& elementsName) noexcept;

    /**
     * Add a new variable to the list
     * @param name of the variable
     * @param elementsName vector containing the name associated to each elements of the variable
     * @return true/false in case of success/failure
     */
    bool addVariable(const std::string& name,
                     const std::vector<std::string>& elementsName) noexcept;

    /**
     * Get a variable from the list
     * @param name of the variable
     * @return the variable description associated to the variable
     */
    const VariableDescription& getVariable(const std::string& name) const noexcept;

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

    /**
     * Get the string representation of the handler.
     * @return a string containing the name of all the variables and their description.
     */
    std::string toString() const noexcept;

    /**
     * Clear the content of the handler
     */
    void clear() noexcept;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_VARIABLES_HANDLER_H
