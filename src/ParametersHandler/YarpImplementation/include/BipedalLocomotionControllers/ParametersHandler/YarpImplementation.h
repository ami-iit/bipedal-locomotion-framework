/**
 * @file YarpImplementation.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_PARAMETERS_HANDLER_YARP_IMPLEMENTATION_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_PARAMETERS_HANDLER_YARP_IMPLEMENTATION_H

// std
#include <memory>

// YARP
#include <yarp/os/Searchable.h>
#include <yarp/os/Property.h>

#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotionControllers
{
namespace ParametersHandler
{

template <typename T>
struct is_string : public std::disjunction<std::is_same<char*, typename std::decay<T>::type>,
                                           std::is_same<const char*, typename std::decay<T>::type>,
                                           std::is_same<std::string, typename std::decay<T>::type>>
{
};

/**
 * Parameters handler interface. Yarp Implementation (Curiously recurring
 * template pattern)
 */
class YarpImplementation : public IParametersHandler<YarpImplementation>
{

    yarp::os::Property m_container; /**< Bottle object */

public:
    /**
     * Constructor.
     * @param searchable reference to a searchable object. The object is copied inside the Handler
     */
    YarpImplementation(const yarp::os::Searchable& searchable);

    /**
     * Constructor.
     */
    YarpImplementation() = default;

    /**
     * Get a parameter from the handler.
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @tparam T type of the parameter
     * @return true/false in case of success/failure
     */
    template <typename T> bool getParameter(const std::string& parameterName, T& parameter) const;

    /**
     * Set a parameter in the handler.
     * @param parameterName name of the parameter
     * @param parameter parameter
     * @tparam T type of the parameter
     * @return true/false in case of success/failure
     */
    template <typename T> void setParameter(const std::string& parameterName, const T& parameter);

    /**
     * Get a Group from the handler.
     * @param name name of the group
     * @return A pointer to IParametersHandler, If the group is not found a new empty object is
     * created and returned
     */
    std::unique_ptr<IParametersHandler<YarpImplementation>> getGroup(const std::string& name) const;

    /**
     * Operator << overloading
     * @param os Output stream objects
     * @param handler reference to the interface
     * @return a reference to an Output stream objects
     */
    friend std::ostream& operator<<(std::ostream& os, const YarpImplementation& hanlder);

    /**
     * Destructor
     */
    ~YarpImplementation() = default;
};
} // namespace ParametersHandler
} // namespace BipedalLocomotionControllers

#include "YarpImplementation.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_YARP_UTILITIES_HELPER_H
