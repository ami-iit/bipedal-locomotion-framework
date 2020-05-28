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

#include <BipedalLocomotion/System/FixStepIntegrator.h>

namespace BipedalLocomotion
{
namespace System
{

/**
 * Forward Euler integration method.
 * @tparam DynamicalSystemDerived a class derived from DynamicalSystem
 * @warning We assume that the vectors / matrices contained in the DynamicalSystemDerived::StateType
 * and DynamicalSystemDerived::StateDerivativeType can be converted into an Eigen::Map using
 * iDynTree::toEigen() function
 */
template <typename DynamicalSystemDerived>
class ForwardEuler : public FixStepIntegrator<DynamicalSystemDerived>
{
    typename DynamicalSystemDerived::StateDerivativeType m_computationalBuffer;

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

        iDynTree::toEigen(std::get<I>(x)) += iDynTree::toEigen(std::get<I>(dx)) * dT;

        if constexpr (std::is_same<typename std::remove_reference<decltype(std::get<I>(x))>::type,
                                   iDynTree::Rotation>::value)
        {
            Eigen::Matrix3d baseRotationEigen = iDynTree::toEigen(std::get<I>(x));
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(baseRotationEigen,
                                                  Eigen::ComputeFullU | Eigen::ComputeFullV);

            iDynTree::toEigen(std::get<I>(x)) = svd.matrixU() * svd.matrixV().transpose();
        }

        addArea<I + 1>(dx, dT, x);
    }

    /**
     * Integrate one step.
     * @param t0 initial time.
     * @param dT sampling time.
     * @param x0 initial state.
     * @param x state at t0 + dT.
     * @return true in case of success, false otherwise.
     */
    bool oneStepIntegration(double t0,
                            double dT,
                            const typename DynamicalSystemDerived::StateType& x0,
                            typename DynamicalSystemDerived::StateType& x) final;

public:
    /**
     * Constructor
     * @param dT the sampling time
     */
    ForwardEuler(const double& dT)
        : FixStepIntegrator<DynamicalSystemDerived>(dT)
    {
    }
    ~ForwardEuler() = default;
};

} // namespace System
} // namespace BipedalLocomotion

#include <BipedalLocomotion/System/ForwardEuler.tpp>

#endif // BIPEDAL_LOCOMOTION_SYSTEM_FORWARD_EULER_H
