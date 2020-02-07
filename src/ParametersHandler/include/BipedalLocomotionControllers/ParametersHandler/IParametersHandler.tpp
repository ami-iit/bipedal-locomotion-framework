/**
 * @file IParametersHandler.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

namespace BipedalLocomotionControllers
{
namespace ParametersHandler
{
template <class Derived>
template <typename T>
bool IParametersHandler<Derived>::getParameter(const std::string& parameterName, T& parameter) const
{
    return static_cast<const Derived*>(this)->getParameter(parameterName, parameter);
}

template <class Derived>
std::unique_ptr<IParametersHandler<Derived>>
IParametersHandler<Derived>::getGroup(const std::string& groupName) const
{
    return static_cast<const Derived*>(this)->getGroup(groupName);
}

template <class Derived>
std::ostream& operator<<(std::ostream& os, const IParametersHandler<Derived>&  handler)
{
    return operator<<(os, static_cast<const Derived&>(handler));
}

} // namespace ParametersHandler
} // namespace BipedalLocomotionController
