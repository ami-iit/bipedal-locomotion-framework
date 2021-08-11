/**
 * @file TomlImplementation.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_TOML_IMPLEMENTATION_H
#define BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_TOML_IMPLEMENTATION_H

// std
#include <memory>
#include <string>
#include <unordered_map>

// TOML
#include <toml++/toml.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace ParametersHandler
{

/**
 * TomlImplementation Toml implementation of the IParametersHandler interface
 */
class TomlImplementation : public IParametersHandler
{
    toml::table m_container; /**< Main container */

    /** Map containing pointers to the (asked) groups */
    std::unordered_map<std::string, std::shared_ptr<TomlImplementation>> m_lists;

    /**
     * Private implementation of getParameter
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @tparam T type of the parameter
     * @return true/false in case of success/failure
     */
    template <typename T>
    bool getParameterPrivate(const std::string& parameterName, T& parameter) const;

    /**
     * Private implementation of setParameter
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @tparam T type of the parameter
     */
    template <typename T>
    void setParameterPrivate(const std::string& parameterName, const T& parameter);

    /**
     * Clone the content of the handler
     * @return A pointer to TomlImplementation containing the content of the handler.
     */
    std::shared_ptr<TomlImplementation> clonePrivate() const;

public:
    /**
     * Constructor.
     * @param container reference to a searchable object. The object is copied inside the
     * Handler
     */
    TomlImplementation(const toml::table& container);

    /**
     * Constructor.
     */
    TomlImplementation() = default;

    /**
     * Set the handler from an object.
     * @param container The object to copy
     */
    void set(const toml::table& container);

    /**
     * Set the handler from a file.
     * @param filename the name of the file that should be loaded.
     * @return true/false in case of success/failure.
     */
    bool setFromFile(const std::string& filename);

    /**
     * Get a parameter [int]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */
    bool getParameter(const std::string& parameterName, int& parameter) const final;

    /**
     * Get a parameter [double]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */
    bool getParameter(const std::string& parameterName, double& parameter) const final;

    /**
     * Get a parameter [std::string]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */
    bool getParameter(const std::string& parameterName, std::string& parameter) const final;

    /**
     * Get a parameter [bool]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */

    bool getParameter(const std::string& parameterName, bool& parameter) const final;

    /**
     * Get a parameter [std::vector<bool>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */
    bool getParameter(const std::string& parameterName, std::vector<bool>& parameter) const final;

    /**
     * Get a parameter [GenericContainer::Vector<int>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */
    bool getParameter(const std::string& parameterName, GenericContainer::Vector<int>::Ref parameter) const final;

    /**
     * Get a parameter [GenericContainer::Vector<double>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */
    bool getParameter(const std::string& parameterName, GenericContainer::Vector<double>::Ref parameter) const final;

    /**
     * Get a parameter [GenericContainer::Vector<std::string>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */
    bool getParameter(const std::string& parameterName, GenericContainer::Vector<std::string>::Ref parameter) const final;

    /**
     * Set a parameter [int]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    void setParameter(const std::string& parameterName, const int& parameter) final;

    /**
     * Set a parameter [double]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    void setParameter(const std::string& parameterName, const double& parameter) final;

    /**
     * Set a parameter [std::string]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    void setParameter(const std::string& parameterName, const std::string& parameter) final;

    /**
     * Set a parameter [const char*]
     * @note this is required because of
     * https://www.bfilipek.com/2019/07/surprising-conversions-char-bool.html
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    void setParameter(const std::string& parameterName, const char* parameter) final;

    /**
     * Set a parameter [bool]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    void setParameter(const std::string& parameterName, const bool& parameter) final;

    /**
     * Set a parameter [std::vector<bool>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    void setParameter(const std::string& parameterName, const std::vector<bool>& parameter) final;

    /**
     * Set a parameter [GenericContainer::Vector<int>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    void setParameter(const std::string& parameterName, const GenericContainer::Vector<const int>::Ref parameter) final;

    /**
     * Set a parameter [GenericContainer::Vector<double>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    void setParameter(const std::string& parameterName, const GenericContainer::Vector<const double>::Ref parameter) final;

    /**
     * Set a parameter [GenericContainer::Vector<std::string>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    void setParameter(const std::string& parameterName, const GenericContainer::Vector<const std::string>::Ref parameter) final;

    /**
     * Get a Group from the handler.
     * @param name name of the group
     * @return A pointer to IParametersHandler, if the group is not found the weak pointer cannot
     * be locked
     */
    weak_ptr getGroup(const std::string& name) const final;

    /**
     * Set a new group on the handler.
     * @param name name of the group
     * @param newGroup shared pointer to the new group
     */
    bool setGroup(const std::string& name, shared_ptr newGroup) final;

    /**
     * Return a standard text representation of the content of the object.
     * @return a string containing the standard text representation of the content of the object.
     */
    std::string toString() const final;

    /**
     * Check if the handler contains parameters
     * @return true if the handler does not contain any parameters, false otherwise
     */
    bool isEmpty() const final;

    /**
     * Clears the handler from all the parameters
     */
    void clear() final;

    /**
     * Clone the content of the content.
     * @return a IParametersHandler::shared_ptr clone of the current handler.
     * @warning
     */
    shared_ptr clone() const final;

    /**
     * Destructor
     */
    ~TomlImplementation() = default;
};

/**
 * Private implementation of setParameter
 * @param parameterName name of the parameter
 * @param parameter parameter
 * @note The specialization is required because std::vector<bool> is not a container and the
 * operator[] does not return a bool&
 */
template <>
void TomlImplementation::setParameterPrivate<std::vector<bool>>(const std::string& parameterName,
                                                                const std::vector<bool>& parameter);
} // namespace ParametersHandler
} // namespace BipedalLocomotion

#include "TomlImplementation.tpp"

#endif // BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_TOML_IMPLEMENTATION_H
