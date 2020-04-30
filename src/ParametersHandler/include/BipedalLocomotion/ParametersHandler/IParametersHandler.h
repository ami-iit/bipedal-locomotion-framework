/**
 * @file IParametersHandler.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_IPARAMETERS_HANDLER_H
#define BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_IPARAMETERS_HANDLER_H

#include <memory>
#include <string>
#include <vector>

#include <BipedalLocomotion/GenericContainer/Vector.h>
#include <BipedalLocomotion/GenericContainer/TemplateHelpers.h>

namespace BipedalLocomotion
{
namespace ParametersHandler
{

/**
 * Parameters handler interface.
 * @tparam Derived type of the Derived class. Necessary to implement the Curiously recurring
 * template pattern
 */
class IParametersHandler
{
protected:
    /**
     * Get a parameter [GenericContainer::Vector<int>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */
    virtual bool getParameter(const std::string& parameterName, GenericContainer::Vector<int>& parameter) const = 0;

    /**
     * Get a parameter [GenericContainer::Vector<double>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */
    virtual bool getParameter(const std::string& parameterName, GenericContainer::Vector<double>& parameter) const = 0;

    /**
     * Get a parameter [GenericContainer::Vector<std::string>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */
    virtual bool getParameter(const std::string& parameterName, GenericContainer::Vector<std::string>& parameter) const = 0;

    /**
     * Set a parameter [GenericContainer::Vector<int>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    virtual void setParameter(const std::string& parameterName, const GenericContainer::Vector<const int>& parameter) = 0;

    /**
     * Set a parameter [GenericContainer::Vector<double>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    virtual void setParameter(const std::string& parameterName, const GenericContainer::Vector<const double>& parameter) = 0;

    /**
     * Set a parameter [GenericContainer::Vector<std::string>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    virtual void setParameter(const std::string& parameterName, const GenericContainer::Vector<const std::string>& parameter) = 0;

public:

    using unique_ptr = std::unique_ptr<IParametersHandler>;

    using shared_ptr = std::shared_ptr<IParametersHandler>;

    using weak_ptr = std::weak_ptr<IParametersHandler>;

    /**
     * Get a parameter [int]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */
    virtual bool getParameter(const std::string& parameterName, int& parameter) const = 0;

    /**
     * Get a parameter [double]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */
    virtual bool getParameter(const std::string& parameterName, double& parameter) const = 0;

    /**
     * Get a parameter [std::string]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */
    virtual bool getParameter(const std::string& parameterName, std::string& parameter) const = 0;

    /**
     * Get a parameter [bool]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */

    virtual bool getParameter(const std::string& parameterName, bool& parameter) const = 0;

    /**
     * Get a parameter [std::vector<bool>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @return true/false in case of success/failure
     */
    virtual bool getParameter(const std::string& parameterName, std::vector<bool>& parameter) const = 0;

    /**
     * Get a vector parameter
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @tparam Vector type of the vector.
     * @warning this method can be only used with vectors that can be converted in a GenericContainer::Vector
     * @return true/false in case of success/failure
     */
    template <class Vector, typename = typename std::enable_if<!GenericContainer::is_vector<Vector>::value &&
        !std::is_same<Vector, std::string>::value && GenericContainer::is_vector_constructible<Vector>::value>::type>
    bool getParameter(const std::string& parameterName,
                      Vector& parameter,
                      GenericContainer::VectorResizeMode mode = GenericContainer::VectorResizeMode::Fixed) const
    {
        auto genericVector = GenericContainer::make_vector(parameter, mode);
        return this->getParameter(parameterName, genericVector);
    }

    /**
     * Set a parameter [int]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    virtual void setParameter(const std::string& parameterName, const int& parameter) = 0;

    /**
     * Set a parameter [double]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    virtual void setParameter(const std::string& parameterName, const double& parameter) = 0;

    /**
     * Set a parameter [std::string]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    virtual void setParameter(const std::string& parameterName, const std::string& parameter) = 0;

    /**
     * Set a parameter [const char*]
     * @note this is required because of
     * https://www.bfilipek.com/2019/07/surprising-conversions-char-bool.html
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    virtual void setParameter(const std::string& parameterName, const char* parameter) = 0;

    /**
     * Set a parameter [bool]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    virtual void setParameter(const std::string& parameterName, const bool& parameter) = 0;


    /**
     * Set a parameter [std::vector<bool>]
     * @param parameterName name of the parameter
     * @param parameter parameter
     */
    virtual void setParameter(const std::string& parameterName, const std::vector<bool>& parameter) = 0;

    /**
     * set a vector parameter
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @tparam Vector type of the vector.
     * @warning this method can be only used with vectors that can be converted in a GenericContainer::Vector
     */
    template <class Vector, typename = typename std::enable_if<!GenericContainer::is_vector<Vector>::value &&
        !std::is_same<Vector, std::string>::value && GenericContainer::is_vector_constructible<Vector>::value>::type>
    void setParameter(const std::string& parameterName, const Vector& parameter)
    {
        auto genericVector = GenericContainer::make_vector(parameter);
        return this->setParameter(parameterName, genericVector);
    }

    /**
     * Get a Group from the handler.
     * @param name name of the group
     * @return A pointer to IParametersHandler, if the group is not found the weak pointer cannot
     * be locked
     * @warning Please implement the specific version of this method in the Derived class. Please
     * check YarpImplementation::getGroup
     */
    virtual weak_ptr getGroup(const std::string& name) const = 0;

    /**
     * Set a new group on the handler.
     * @param name name of the group
     * @param newGroup shared pointer to the new group
     * @warning Please implement the specific version of this method in the Derived class. Please
     * check YarpImplementation::setGroup
     * @return true/false in case of success/failure
     */
    virtual bool setGroup(const std::string& name, shared_ptr newGroup) = 0;

    /**
     * Return a standard text representation of the content of the object.
     * @return a string containing the standard text representation of the content of the object.
     * @warning Please implement the specific version of this method in the Derived class. Please
     * check YarpImplementation::toString
     */
    virtual std::string toString() const = 0;

    /**
     * Check if the handler contains parameters
     * @return true if the handler does not contain any parameters, false otherwise
     * @warning Please implement the specific version of this method in the Derived class. Please
     * check YarpImplementation::isEmpty
     */
    virtual bool isEmpty() const = 0;

    /**
     * Clears the handler from all the parameters
     * @warning Please implement the specific version of this method in the Derived class. Please
     * check YarpImplementation::clear
     */
    virtual void clear() = 0;

    /**
     * Destructor
     */
    virtual ~IParametersHandler() = default;

};
} // namespace ParametersHandler
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_IPARAMETERS_HANDLER_H
