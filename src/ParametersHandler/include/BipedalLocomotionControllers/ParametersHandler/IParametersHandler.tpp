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
template <typename T>
void IParametersHandler<Derived>::setParameter(const std::string& parameterName, const T& parameter)
{
    return static_cast<Derived*>(this)->setParameter(parameterName, parameter);
}

template <class Derived>
std::unique_ptr<IParametersHandler<Derived>>
IParametersHandler<Derived>::getGroup(const std::string& groupName) const
{
    return static_cast<const Derived*>(this)->getGroup(groupName);
}

template <class Derived> std::string IParametersHandler<Derived>::toString() const
{
    return static_cast<const Derived*>(this)->toString();
}

template <class Derived> bool IParametersHandler<Derived>::isEmpty() const
{
    return static_cast<const Derived*>(this)->isEmpty();
}

template <class Derived>
std::ostream& operator<<(std::ostream& os, const IParametersHandler<Derived>& handler)
{
    return os << handler.toString();
}

} // namespace ParametersHandler
} // namespace BipedalLocomotionControllers
