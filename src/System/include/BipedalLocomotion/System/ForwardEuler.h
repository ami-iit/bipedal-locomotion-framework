/**
 * @file ForwardEuler.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_FORWARD_EULER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_FORWARD_EULER_H

#include <tuple>
#include <type_traits>

#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotion/System/FixedStepIntegrator.h>

namespace BipedalLocomotion
{
namespace System
{

/**
 * Forward Euler integration method.
 * @tparam DynamicalSystemDerived a class derived from DynamicalSystem
 * @warning We assume that the operator + is defined for the objects contained in the
 * DynamicalSystemDerived::StateType and DynamicalSystemDerived::StateDerivativeType.
 */
template <typename DynamicalSystemDerived>
class ForwardEuler : public FixedStepIntegrator<DynamicalSystemDerived>
{
    typename DynamicalSystemDerived::StateDerivativeType m_computationalBufferStateDerivative;
    typename DynamicalSystemDerived::StateType m_computationalBufferState;

    template <std::size_t I = 0, typename... Tp, typename... Td>
    inline typename std::enable_if<I == sizeof...(Tp), void>::type
    addArea(const std::tuple<Tp...>& dx, const double& dT, std::tuple<Td...>& x)
    {
        static_assert(sizeof...(Tp) == sizeof...(Td));
    }

    template <std::size_t I = 0, typename... Tp, typename... Td>
        inline typename std::enable_if < I<sizeof...(Tp), void>::type
        addArea(const std::tuple<Tp...>& dx, const double& dT, std::tuple<Td...>& x)
    {
        static_assert(sizeof...(Tp) == sizeof...(Td));

        std::get<I>(x) += std::get<I>(dx) * dT;
        addArea<I + 1>(dx, dT, x);
    }

    /**
     * Integrate one step.
     * @param t0 initial time.
     * @param dT sampling time.
     * @return true in case of success, false otherwise.
     */
    bool oneStepIntegration(double t0, double dT) final;

public:
    /**
     * Constructor
     * @param dT the sampling time
     */
    ForwardEuler(const double& dT)
        : FixedStepIntegrator<DynamicalSystemDerived>(dT)
    {
    }
    ~ForwardEuler() = default;
};

} // namespace System
} // namespace BipedalLocomotion

#include <BipedalLocomotion/System/ForwardEuler.tpp>

#endif // BIPEDAL_LOCOMOTION_SYSTEM_FORWARD_EULER_H
