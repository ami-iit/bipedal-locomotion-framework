/**
 * @file RK4.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_RK4_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_RK4_H

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
template <typename _DynamicalSystem> class RK4;
}
} // namespace BipedalLocomotion

BLF_DEFINE_INTEGRATOR_STRUCTURE(RK4, _DynamicalSystemType)

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * RK4 implements the runge-kutta 4 integration method.
 * It implements the following equation
 * \f[
 * x_{k+1} = x_k + \frac{1}{6} \left( k_1 + 2 k_2 + 2 k_3 + k_4 \right)
 * \f]
 * where
 * \f[
 * \begin{cases}
 * k_1 = f(x_k, u_k, t_k) \\
 * k_2 = f(x_k + \frac{dt}{2} k_1, u_k, t_k + \frac{dt}{2}) \\
 * k_3 = f(x_k + \frac{dt}{2} k_2, u_k, t_k + \frac{dt}{2}) \\
 * k_4 = f(x_k + dt k_3, u_k, t_k + dt)
 * \end{cases}
 * \f]
 * where \f$x_k\f$ is the state at time \f$t_k\f$, \f$u_k\f$ is the input at time \f$t_k\f$ and
 * \f$dt\f$ is the sampling time.
 * @tparam _DynamicalSystem a class derived from DynamicalSystem
 * @warning We assume that the operator + is defined for the objects contained in the
 * DynamicalSystem::State and DynamicalSystem::StateDerivative.
 * @warning The RK4 integrator is compatible only with tuple containing vectors belonging to the
 * \f$R^n\f$ space. It is not compatible with Lie groups. If you want to integrate a dynamical
 * system defined on a Lie group please use the ForwardEuler integrator.
 */
template <class _DynamicalSystem> class RK4 : public FixedStepIntegrator<RK4<_DynamicalSystem>>
{
public:
    using DynamicalSystem = typename internal::traits<RK4<_DynamicalSystem>>::DynamicalSystem;
    using State = typename internal::traits<RK4<_DynamicalSystem>>::State;
    using StateDerivative = typename internal::traits<RK4<_DynamicalSystem>>::StateDerivative;

private:
    /** Temporary buffer usefully to avoid continuous memory allocation */
    StateDerivative m_k1;
    StateDerivative m_k2;
    StateDerivative m_k3;
    StateDerivative m_k4;

    /** Temporary buffer usefully to avoid continuous memory allocation */
    State m_computationalBufferState1;
    State m_computationalBufferState2;
    State m_computationalBufferState3;
    State m_computationalBufferState4;

    template <std::size_t I = 0>
    inline typename std::enable_if<I == std::tuple_size<State>::value, void>::type
    computeNextState(const StateDerivative& k, const std::chrono::nanoseconds& dT, State& x)
    {
    }

    template <std::size_t I = 0>
    inline typename std::enable_if<(I < std::tuple_size<State>::value), void>::type
    computeNextState(const StateDerivative& k, const std::chrono::nanoseconds& dT, State& x)
    {
        static_assert(std::tuple_size<State>::value == std::tuple_size<StateDerivative>::value);

        using std::get;

        // convert the dT in seconds and compute the next state
        get<I>(x) = (get<I>(k) * std::chrono::duration<double>(dT).count()) + get<I>(x);
        computeNextState<I + 1>(k, dT, x);
    }

    template <std::size_t I = 0>
    inline typename std::enable_if<I == std::tuple_size<State>::value, void>::type
    integrateRK4(const StateDerivative& k1,
                 const StateDerivative& k2,
                 const StateDerivative& k3,
                 const StateDerivative& k4,
                 const std::chrono::nanoseconds& dT,
                 State& x)
    {
    }

    template <std::size_t I = 0>
    inline typename std::enable_if<(I < std::tuple_size<State>::value), void>::type
    integrateRK4(const StateDerivative& k1,
                 const StateDerivative& k2,
                 const StateDerivative& k3,
                 const StateDerivative& k4,
                 const std::chrono::nanoseconds& dT,
                 State& x)
    {
        static_assert(std::tuple_size<State>::value == std::tuple_size<StateDerivative>::value);
        using std::get;

        // convert the dT in seconds
        const double dTInSeconds = std::chrono::duration<double>(dT).count();

        // complete the RK4 integration
        get<I>(x) = get<I>(x)
                    + dTInSeconds / 6 * (get<I>(k1) + 2 * get<I>(k2) + 2 * get<I>(k3) + get<I>(k4));
        this->integrateRK4<I + 1>(k1, k2, k3, k4, dT, x);
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
bool RK4<_DynamicalSystem>::oneStepIntegration(const std::chrono::nanoseconds& t0,
                                               const std::chrono::nanoseconds& dT)
{
    constexpr auto errorPrefix = "[RK4::oneStepIntegration]";
    if (this->m_dynamicalSystem == nullptr)
    {
        log()->error("{} Please specify the dynamical system.", errorPrefix);
        return false;
    }

    // copy the state 4 times since we need to compute the k1, k2, k3 and k4
    this->m_computationalBufferState1 = this->m_dynamicalSystem->getState();
    this->m_computationalBufferState2 = this->m_dynamicalSystem->getState();
    this->m_computationalBufferState3 = this->m_dynamicalSystem->getState();
    this->m_computationalBufferState4 = this->m_dynamicalSystem->getState();

    // evaluate k1
    // k1 = f(x0, u0)
    if (!this->m_dynamicalSystem->dynamics(t0, this->m_k1))
    {
        log()->error("{} Unable to compute the system dynamics while evaluating k1.", errorPrefix);
        return false;
    }

    // evaluate k2
    // k2 = f(x0 + dt / 2 * k1, u0);
    this->computeNextState(this->m_k1, dT / 2, this->m_computationalBufferState2);
    if (!this->m_dynamicalSystem->setState(this->m_computationalBufferState2))
    {
        log()->error("{} Unable to set the new state in the dynamical system required to evaluate "
                     "k2.",
                     errorPrefix);
        return false;
    }

    if (!this->m_dynamicalSystem->dynamics(t0, this->m_k2))
    {
        log()->error("{} Unable to compute the system dynamics while evaluating k2.", errorPrefix);
        return false;
    }

    // evaluate k3
    // k3 = f(x0 + dt / 2 * k2, u0);
    this->computeNextState(this->m_k2, dT / 2, this->m_computationalBufferState3);
    if (!this->m_dynamicalSystem->setState(this->m_computationalBufferState3))
    {
        log()->error("{} Unable to set the new state in the dynamical system required to evaluate "
                     "k3.",
                     errorPrefix);
        return false;
    }

    if (!this->m_dynamicalSystem->dynamics(t0, this->m_k3))
    {
        log()->error("{} Unable to compute the system dynamics while evaluating k3.", errorPrefix);
        return false;
    }

    // evaluate k4
    // k4 = f(x0 + dt * k3, u0);
    this->computeNextState(this->m_k3, dT, this->m_computationalBufferState4);
    if (!this->m_dynamicalSystem->setState(this->m_computationalBufferState4))
    {
        log()->error("{} Unable to set the new state in the dynamical system required to evaluate "
                     "k4.",
                     errorPrefix);
        return false;
    }

    if (!this->m_dynamicalSystem->dynamics(t0, this->m_k4))
    {
        log()->error("{} Unable to compute the system dynamics while evaluating k4.", errorPrefix);
        return false;
    }

    // compute the integration
    this->integrateRK4(this->m_k1,
                       this->m_k2,
                       this->m_k3,
                       this->m_k4,
                       dT,
                       this->m_computationalBufferState1);
    if (!this->m_dynamicalSystem->setState(this->m_computationalBufferState1))
    {
        log()->error("{} Unable to set the new state in the dynamical system.", errorPrefix);
        return false;
    }

    return true;
}

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_RK4_H
