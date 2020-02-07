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

#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotionControllers
{
namespace ParametersHandler
{
/**
 * Parameters handler interface. Yarp Implementation (Curiously recurring
 * template pattern)
 */
class YarpImplementation : public IParametersHandler<YarpImplementation>
{

    const yarp::os::Searchable& m_searchable; /**< Reference to a searchable object */

public:
    /**
     * Constructor.
     * @param searchable reference to a searchable object. The object has to exist from the entire
     * lifetime of YarpImplementation
     */
    YarpImplementation(const yarp::os::Searchable& searchable);

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
    std::unique_ptr<IParametersHandler<YarpImplementation>> getGroup(const std::string& name) const;

    /**
     * Operator << overloading
     * @param os Output stream objects
     * @param handler reference to the interface
     * @return a reference to an Output stream objects
     */
    friend std::ostream& operator<<(std::ostream& os, const YarpImplementation& hanlder);
};
} // namespace ParametersHandler
} // namespace BipedalLocomotionControllers

#include "YarpImplementation.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_YARP_UTILITIES_HELPER_H
