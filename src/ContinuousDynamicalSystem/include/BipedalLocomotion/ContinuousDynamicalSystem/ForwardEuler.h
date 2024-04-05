/**
 * @file ForwardEuler.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FORWARD_EULER_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FORWARD_EULER_H

#include <chrono>
#include <tuple>
#include <type_traits>

#include <iDynTree/EigenHelpers.h>

#include <BipedalLocomotion/ContinuousDynamicalSystem/FixedStepIntegrator.h>
#include <BipedalLocomotion/GenericContainer/NamedTuple.h>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{
template <typename _DynamicalSystem> class ForwardEuler;
}
}

BLF_DEFINE_INTEGRATOR_STRUCTURE(ForwardEuler, _DynamicalSystemType)

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{


/**
 * Forward Euler integration method.
 * @tparam _DynamicalSystem a class derived from DynamicalSystem
 * @warning We assume that the operator + is defined for the objects contained in the
 * DynamicalSystem::State and DynamicalSystem::StateDerivative.
 * @warning The ForwardEuler integrator is compatible with Lie groups defined by the `manif` library.
 * Since the _box-plus_ operator is not commutative for a Lie group, here we consider only manifold
 * left plus
 * \f[
 * X + \psi = X \circ  \exp(\psi)
 * \f]
 * where \f$X\f$ belongs to a Lie group and \f$\psi\f$ belongs to the tangent space.
 */
template <class _DynamicalSystem>
class ForwardEuler : public FixedStepIntegrator<ForwardEuler<_DynamicalSystem>>
{
public:
    using DynamicalSystem = typename internal::traits<ForwardEuler<_DynamicalSystem>>::DynamicalSystem;
    using State = typename internal::traits<ForwardEuler<_DynamicalSystem>>::State;
    using StateDerivative = typename internal::traits<ForwardEuler<_DynamicalSystem>>::StateDerivative;

private:

    /** Temporary buffer usefully to avoid continuous memory allocation */
    StateDerivative m_computationalBufferStateDerivative;

    /** Temporary buffer usefully to avoid continuous memory allocation */
    State m_computationalBufferState;

    template <std::size_t I = 0>
    inline typename std::enable_if<I == std::tuple_size<State>::value, void>::type
    addArea(const StateDerivative& dx, const std::chrono::nanoseconds& dT, State& x)
    {
        static_assert(std::tuple_size<State>::value == std::tuple_size<StateDerivative>::value);
    }

    template <std::size_t I = 0>
    inline typename std::enable_if<(I < std::tuple_size<State>::value), void>::type
    addArea(const StateDerivative& dx, const std::chrono::nanoseconds& dT, State& x)
    {
        static_assert(std::tuple_size<State>::value == std::tuple_size<StateDerivative>::value);

        // the order matters since we assume that all the velocities are left trivialized.
        using std::get;

        // convert the dT in seconds
        get<I>(x) = (get<I>(dx) * std::chrono::duration<double>(dT).count()) + get<I>(x);
        addArea<I + 1>(dx, dT, x);
    }

public:
    /**
     * Integrate one step.
     * @param t0 initial time.
     * @param dT sampling time.
     * @return true in case of success, false otherwise.
     */
    bool oneStepIntegration(const std::chrono::nanoseconds& t0, const std::chrono::nanoseconds& dT);
};

template <class _DynamicalSystem>
bool ForwardEuler<_DynamicalSystem>::oneStepIntegration(const std::chrono::nanoseconds& t0,
                                                        const std::chrono::nanoseconds& dT)
{
    constexpr auto errorPrefix = "[ForwardEuler::oneStepIntegration]";
    if (this->m_dynamicalSystem == nullptr)
    {
        log()->error("{} Please specify the dynamical system.", errorPrefix);
        return false;
    }

    if (!this->m_dynamicalSystem->dynamics(t0, this->m_computationalBufferStateDerivative))
    {
        log()->error("{} Unable to compute the system dynamics.", errorPrefix);
        return false;
    }

    // x = dT * dx + x0
    this->m_computationalBufferState = this->m_dynamicalSystem->getState();
    this->addArea(this->m_computationalBufferStateDerivative, dT, this->m_computationalBufferState);

    if (!this->m_dynamicalSystem->setState(this->m_computationalBufferState))
    {
        log()->error("{} Unable to set the new state in the dynamical system.", errorPrefix);
        return false;
    }

    return true;
}

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FORWARD_EULER_H
