/**
 * @file Integrator.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_INTEGRATOR_TPP
#define BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_INTEGRATOR_TPP

#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotionControllers/Simulator/Integrator.h>

namespace BipedalLocomotionControllers
{
namespace Simulator
{

template <typename T> Integrator<T>::Integrator(const double dT, const T& y0)
{
    // the integral time must be positive
    assert(dT > 0);

    // the copy assignment is required to ensure that m_xOld and m_y have the same size. Since this
    // class is compatible also with matrices, m_xOld.resize(y0.size()) is not sufficient. Once
    // m_xOld have a well defined size, it is reset to zero
    m_xOld = y0;
    m_xOld.zero();

    m_y = y0;
    m_dT = dT;
}

template <typename T> Integrator<T>::Integrator(const double dT)
{
    // the integral time must be positive
    assert(dT > 0);

    m_dT = dT;
}

template <typename T> void Integrator<T>::reset(const T& y0)
{
    // the copy assignment is required to ensure that m_xOld and m_y have the same size. Since this
    // class is compatible also with matrices, m_xOld.resize(y0.size()) is not sufficient. Once
    // m_xOld have a well defined size, it is reset to zero
    m_xOld = y0;
    m_xOld.zero();

    m_y = y0;
}

template <typename T> const T& Integrator<T>::integrate(const T& x)
{
    using iDynTree::toEigen;

    // Tustin formula implementation
    toEigen(m_y) = toEigen(m_y) + (toEigen(x) + toEigen(m_xOld)) * (m_dT / 2);
    m_xOld = x;

    return m_y;
}

template <typename T> const T& Integrator<T>::get() const
{
    return m_y;
}

} // namespace Simulator
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_INTEGRATOR_TPP
