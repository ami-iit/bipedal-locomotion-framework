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

    using unique_ptr = std::unique_ptr<IParametersHandler<Derived>>;

    /**
     * Get a parameter from the handler.
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @tparam T type of the parameter
     * @return true/false in case of success/failure
     * @warning Please implement the specific version of this method in the Derived class. Please
     * check YarpImplementation::getParameter
     */
    template <typename T> bool getParameter(const std::string& parameterName, T& parameter) const;

    /**
     * Set a parameter in the handler.
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @tparam T type of the parameter
     * @warning Please implement the specific version of this method in the Derived class. Please
     * check YarpImplementation::setParameter
     */
    template <typename T> void setParameter(const std::string& parameterName, const T& parameter);

    /**
     * Get a Group from the handler.
     * @param name name of the group
     * @return A pointer to IParametersHandler, If the group is not found the pointer is equal to
     * nullptr
     * @warning Please implement the specific version of this method in the Derived class. Please
     * check YarpImplementation::getGroup
     */
    std::unique_ptr<IParametersHandler<Derived>> getGroup(const std::string& name) const;

    /**
     * Return a standard text representation of the content of the object.
     * @return a string containing the standard text representation of the content of the object.
     * @warning Please implement the specific version of this method in the Derived class. Please
     * check YarpImplementation::toString
     */
    std::string toString() const;

    /**
     * Check if the handler contains parameters
     * @return true of the the handler does not contains any parameters, false otherwise
     * @warning Please implement the specific version of this method in the Derived class. Please
     * check YarpImplementation::isEmpty
     */
    bool isEmpty() const;

    /**
     * Operator << overloading
     * @param os Output stream objects
     * @param handler reference to the interface
     * @tparam U type of the derived class
     * @return a reference to an Output stream objects
     */
    template <typename U>
    friend std::ostream& operator<<(std::ostream& os, const IParametersHandler<U>& hanlder);

    /**
     * Destructor
     */
    virtual ~IParametersHandler() = default;
};
} // namespace ParametersHandler
} // namespace BipedalLocomotionControllers

#include "IParametersHandler.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_PARAMETERS_HANDLER_IPARAMETERS_HANDLER_H
