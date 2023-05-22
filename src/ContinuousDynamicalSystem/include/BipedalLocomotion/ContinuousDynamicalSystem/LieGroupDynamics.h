/**
 * @file LieGroupDynamics.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_LIE_GROUPS_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_LIE_GROUPS_DYNAMICS_H

#include <memory>
#include <vector>

#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/GenericContainer/NamedTuple.h>

#include <manif/manif.h>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

// Forward declare for type traits specialization
template <typename _Derived>
class LieGroupDynamics;

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

namespace BipedalLocomotion::ContinuousDynamicalSystem::internal
{
template <typename _Derived>
struct traits<BipedalLocomotion::ContinuousDynamicalSystem::LieGroupDynamics<_Derived>>
{
    using Tangent = typename _Derived::Tangent;
    using State = GenericContainer::named_tuple<BLF_NAMED_PARAM(LieGroup, _Derived)>;
    using StateDerivative = GenericContainer::named_tuple<BLF_NAMED_PARAM(Tangent, Tangent)>;
    using Input = GenericContainer::named_tuple<BLF_NAMED_PARAM(Tangent, Tangent)>;
    using DynamicalSystem = LieGroupDynamics<_Derived>;
};
} // namespace BipedalLocomotion::ContinuousDynamicalSystem::internal

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * LieGroupDynamics describes the dynamics of a LieGroup.
 * In details, given an element of a Lie group\f$X \in G\f$ and a vector of its Lie Algebra
 * \f$ x \in \mathrm{g}\f$, LieGroupDynamics implements
 * \f[
 * \dot{X} = x X
 * \f]
 * where \f$\dot{X} \in T_X G\f$ with \f$T_X G\f$ is the Tangent space of \f$G\f$ at \f$X\f$.

 * The LieGroupDynamics inherits from a generic DynamicalSystem where the State is
 * described by a BipedalLocomotion::GenericContainer::named_tuple
 * |    Name    |        Type       |                Description                 |
 * |:----------:|:-----------------:|:------------------------------------------:|
 * | `LieGroup` |     `_Derived`    |  The Lie group considered by the dynamics  |
 *
 * The `StateDerivative` is described by a BipedalLocomotion::GenericContainer::named_tuple
 * |    Name    |         Type        |                     Description                      |
 * |:----------:|:-------------------:|:----------------------------------------------------:|
 * | `Tangent`  | `_Derived::Tangent` |  Element of the tangent vector in Left trivialized   |
 *
 * The `Input` is described by a BipedalLocomotion::GenericContainer::named_tuple
 * |    Name    |         Type        |                     Description                      |
 * |:----------:|:-------------------:|:----------------------------------------------------:|
 * | `Tangent`  | `_Derived::Tangent` |  Element of the tangent vector in Left trivialized   |
 */
template <typename _Derived>
class LieGroupDynamics : public DynamicalSystem<LieGroupDynamics<_Derived>>
{

public:
    using State = typename DynamicalSystem<LieGroupDynamics<_Derived>>::State;
    using Input = typename DynamicalSystem<LieGroupDynamics<_Derived>>::Input;
    using StateDerivative = typename DynamicalSystem<LieGroupDynamics<_Derived>>::StateDerivative;

private:
    State m_state;
    Input m_controlInput;

public:
    /**
     * Set the state of the dynamical system.
     * @param state tuple containing a const reference to the state elements.
     * @return true in case of success, false otherwise.
     */
    bool setState(const State& state)
    {
        m_state = state;
        return true;
    }

    /**
     * Get the state to the dynamical system.
     * @return the current state of the dynamical system
     */
    const State& getState() const
    {
        return m_state;
    }

    /**
     * Set the control input to the dynamical system.
     * @param controlInput the value of the control input used to compute the system dynamics.
     * @return true in case of success, false otherwise.
     */
    bool setControlInput(const Input& controlInput)
    {
        m_controlInput = controlInput;
        return true;
    }

    /**
     * Computes the floating based system dynamics. It return \f$f(x, u, t)\f$.
     * @note The control input and the state have to be set separately with the methods
     * setControlInput and setState.
     * @param time the time at witch the dynamics is computed.
     * @param stateDynamics tuple containing a reference to the element of the state derivative
     * @return true in case of success, false otherwise.
     */
    bool dynamics(const std::chrono::nanoseconds& time, StateDerivative& stateDerivative)
    {
        using namespace BipedalLocomotion::GenericContainer::literals;

        // get the state derivative
        auto& baseAngularVelocity = stateDerivative.template get_from_hash<"Tangent"_h>();

        baseAngularVelocity = m_controlInput.template get_from_hash<"Tangent"_h>();

        return true;
    }
};

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_LIE_GROUPS_DYNAMICS_H
