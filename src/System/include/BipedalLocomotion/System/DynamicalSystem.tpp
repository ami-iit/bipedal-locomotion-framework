/**
 * @file DynamicalSystem.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_DYNAMICAL_SYSTEM_TPP
#define BIPEDAL_LOCOMOTION_SYSTEM_DYNAMICAL_SYSTEM_TPP

#include <BipedalLocomotion/System/DynamicalSystem.h>

namespace BipedalLocomotion
{
namespace System
{

template <typename... StateTypes, typename... StateDerivativeTypes, typename... InputTypes>
bool DynamicalSystem<std::tuple<StateTypes...>,
                     std::tuple<StateDerivativeTypes...>,
                     std::tuple<InputTypes...>>::setInitialState(const StateType& initialState)
{
    m_initialState = initialState;
    return true;
}

template <typename... StateTypes, typename... StateDerivativeTypes, typename... InputTypes>
bool DynamicalSystem<std::tuple<StateTypes...>,
                     std::tuple<StateDerivativeTypes...>,
                     std::tuple<InputTypes...>>::setControlInput(const InputType& controlInput)
{
    m_controlInput = controlInput;
    return true;
}

template <typename... StateTypes, typename... StateDerivativeTypes, typename... InputTypes>
bool DynamicalSystem<std::tuple<StateTypes...>,
                     std::tuple<StateDerivativeTypes...>,
                     std::tuple<InputTypes...>>::
    initalize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    return true;
}

template <typename... StateTypes, typename... StateDerivativeTypes, typename... InputTypes>
const std::tuple<StateTypes...>& DynamicalSystem<std::tuple<StateTypes...>,
                                                 std::tuple<StateDerivativeTypes...>,
                                                 std::tuple<InputTypes...>>::getInitialState() const
{
    return m_initialState;
}

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_DYNAMICAL_SYSTEM_TPP
