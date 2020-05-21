/**
 * @file DynamicalSystem.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_DYNAMICAL_SYSTEM_H
#define BIPEDAL_LOCOMOTION_SYSTEM_DYNAMICAL_SYSTEM_H

#include <memory>
#include <tuple>

#include <BipedalLocomotion/GenericContainer/TemplateHelpers.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace System
{

// Declaration of a template
template <typename StateTypeList, typename DerivativeTypeList, typename InputTypeList>
class DynamicalSystem
{
    static_assert(dependent_false<StateTypeList>::value,
                  "Unable to create the following dynamical system.");
};

/**
 * DynamicalSystem defines a continuos time dynamical system, i.e. \f$\dot{x}=f(x, u, t)\f$. Please
 * inherit publicly from this class  in order to define your custom dynamical system.
 * @tparam StateTypes types used for describing the state (i.e. it may be a list of
 * vectors/matrices).
 * @tparam StateDerivativeTypes types used for describing the state derivative (i.e. it may be a
 * list of vectors/matrices).
 * @tparam InputTypes types used for describing the input (i.e. it may be a list
 * of vectors/matrices or in general classes).
 */
template <typename... StateTypes, typename... StateDerivativeTypes, typename... InputTypes>
class DynamicalSystem<std::tuple<StateTypes...>,
                      std::tuple<StateDerivativeTypes...>,
                      std::tuple<InputTypes...>>
{
public:

    using StateType = std::tuple<StateTypes...>;
    using StateDerivativeType = std::tuple<StateDerivativeTypes...>;
    using InputType = std::tuple<InputTypes...>;

protected:
    StateType m_initialState;
    InputType m_controlInput;

public:

    virtual bool initalize(std::weak_ptr<ParametersHandler::IParametersHandler> handler);

    /**
     * Set the initial state to the dynamical system.
     * @notice In principle, there is no need to override this method. This value is stored in an
     * internal buffer.
     * @param initialState the initial state value.
     * @return true in case of success, false otherwise.
     */
    virtual bool setInitialState(const StateType& initialState);

    /**
     * Get the initial state to the dynamical system.
     * @notice In principle, there is no need to override this method. This value is stored in an
     * internal buffer.
     * @return the initial state of the dynamical system
     */
    virtual const StateType & getInitialState() const;

    /**
     * Set the control input to the dynamical system.
     * @note In principle, there is no need to override this method. This value is stored in an
     * internal buffer.
     * @param controlInput the value of the control input used to compute the system dynamics.
     * @return true in case of success, false otherwise.
     */
    virtual bool setControlInput(const InputType& controlInput);

    /**
     * Computes the system dynamics. It return \f$f(x, u, t)\f&.
     * @note The control input has to be set separately with the method setControlInput.
     * @param state tuple containing a const reference to the state elements.
     * @param time the time at witch the dynamics is computed.
     * @param stateDynamics tuple containing a reference to the element of the state derivative
     * @return true in case of success, false otherwise.
     */
    virtual bool dynamics(const StateType& state,
                          const double& time,
                          StateDerivativeType& stateDerivative) = 0;

    /**
     * Destructor
     */
    ~DynamicalSystem() = default;
};

} // namespace System
} // namespace BipedalLocomotion

#include <BipedalLocomotion/System/DynamicalSystem.tpp>

#endif // BIPEDAL_LOCOMOTION_SYSTEM_DYNAMICAL_SYSTEM_H
