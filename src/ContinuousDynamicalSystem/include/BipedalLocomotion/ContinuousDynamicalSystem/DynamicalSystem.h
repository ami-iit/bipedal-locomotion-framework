/**
 * @file DynamicalSystem.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_DYNAMICAL_SYSTEM_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_DYNAMICAL_SYSTEM_H

#include <chrono>
#include <memory>
#include <tuple>

#include <BipedalLocomotion/ContinuousDynamicalSystem/impl/traits.h>
#include <BipedalLocomotion/GenericContainer/NamedTuple.h>
#include <BipedalLocomotion/GenericContainer/TemplateHelpers.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * DynamicalSystem defines a continuous time dynamical system, i.e. \f$\dot{x}=f(x, u, t)\f$. Please
 * inherit publicly from this class in order to define your custom dynamical system.
 * Just be sure to call after your class definition
 * #BLF_DEFINE_CONTINUOUS_DYNAMICAL_SYSTEM_INTERAL_STRUCTURE()
 * For instance
 * \code{.cpp}
 * namespace BipedalLocomotion
 * {
 * namespace ContinuousDynamicalSystem
 * {
 * // forward declaration
 * class Foo;
 * }
 * }
 * // Here we define the internal structure of the Foo. Notice that the number of types contained in
 * // the FooStateType must be equal to the number of FooStateDerivetiveType (This is required by the integrator class)
 *
 * using FooStateType = Eigen::Vector2d;
 * using FooStateDerivativeType = Eigen::Vector2d;
 * using FooInputType = Eigen::Vector2d;
 * BLF_DEFINE_CONTINUOUS_DYNAMICAL_SYSTEM_INTERAL_STRUCTURE(Foo, (FooStateType),
 *                                                               (FooStateDerivativeType),
 *                                                               (FooInputType))
 * namespace BipedalLocomotion
 * {
 * namespace ContinuousDynamicalSystem
 * {
 * // class definition
 * class Foo : public DynamicalSystem<Foo>
 * {
 *    ...
 * }
 * }
 * }
 * \endcode
 */
template <class _Derived> class DynamicalSystem
{
    constexpr _Derived& derived()
    {
        return *static_cast<_Derived*>(this);
    }
    constexpr const _Derived& derived() const
    {
        return *static_cast<const _Derived*>(this);
    }

public:
    using State = typename internal::traits<_Derived>::State; /**< State space type */
    using StateDerivative = typename internal::traits<_Derived>::StateDerivative; /**< State space
                                                                                     derivative type
                                                                                   */
    using Input = typename internal::traits<_Derived>::Input; /**< Input type */

    static_assert(
        (is_specialization<State, std::tuple>::value
         || is_specialization<State, ::BipedalLocomotion::GenericContainer::named_tuple>::value),
        "State must specialize std::tuple or ::BipedalLocomotion::GenericContainer::named_tuple");

    static_assert((is_specialization<StateDerivative, std::tuple>::value
                   || is_specialization<StateDerivative,
                                        ::BipedalLocomotion::GenericContainer::named_tuple>::value),
                  "StateDerivative must specialize std::tuple or "
                  "::BipedalLocomotion::GenericContainer::named_tuple");

    static_assert(
        (is_specialization<Input, std::tuple>::value
         || is_specialization<Input, ::BipedalLocomotion::GenericContainer::named_tuple>::value),
        "Input must specialize std::tuple or ::BipedalLocomotion::GenericContainer::named_tuple");

    /**
     * Initialize the Dynamical system.
     * @param handler pointer to the parameter handler.
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);

    /**
     * Set the state of the dynamical system.
     * @note In principle, there is no need to override this method. This value is stored in an
     * internal buffer.
     * @param state tuple containing a const reference to the state elements.
     * @return true in case of success, false otherwise.
     */
    bool setState(const State& state);

    /**
     * Get the state to the dynamical system.
     * @return the current state of the dynamical system
     */
    const State& getState() const;

    /**
     * Set the control input to the dynamical system.
     * @note In principle, there is no need to override this method. This value is stored in an
     * internal buffer.
     * @param controlInput the value of the control input used to compute the system dynamics.
     * @return true in case of success, false otherwise.
     */
    bool setControlInput(const Input& controlInput);

    /**
     * Computes the system dynamics. It return \f$f(x, u, t)\f$.
     * @note The control input and the state have to be set separately with the methods
     * setControlInput and setState.
     * @param time the time at witch the dynamics is computed.
     * @param stateDynamics tuple containing a reference to the element of the state derivative
     * @warning Please implement the function in your custom dynamical system.
     * @return true in case of success, false otherwise.
     */
    bool dynamics(const std::chrono::nanoseconds& time, StateDerivative& stateDerivative);
};

template <class _Derived>
bool DynamicalSystem<_Derived>::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    return this->derived().initialize(handler);
}

template <class _Derived> bool DynamicalSystem<_Derived>::setState(const State& state)
{
    return this->derived().setState(state);
}

template <class _Derived>
const typename DynamicalSystem<_Derived>::State& DynamicalSystem<_Derived>::getState() const
{
    return this->derived().getState();
}

template <class _Derived>
bool DynamicalSystem<_Derived>::setControlInput(const typename DynamicalSystem<_Derived>::Input& controlInput)
{
    return this->derived().setControlInput(controlInput);
}

template <class _Derived>
bool DynamicalSystem<_Derived>::dynamics(const std::chrono::nanoseconds& time, StateDerivative& stateDerivative)
{
    return this->derived().dynamics(time, stateDerivative);
}

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_DYNAMICAL_SYSTEM_H
