/**
 * @file Integrator.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include "Integrator.h"

#include <iDynTree/Core/EigenHelpers.h>

namespace BipedalLocomotionControllers
{
namespace Simulator
{

template <typename T> Integrator<T>::Integrator(const double dT, const T& y0)
{
    // dim = (unsigned int)y0.length();
    // x_old.resize(dim, 0.0);
    m_xOld = y0;
    m_xOld.zero();
    m_y = y0;
    m_dT = dT;
}

template <typename T> Integrator<T>::Integrator(const double dT)
{
    m_dT = dT;
}

template <typename T> void Integrator<T>::reset(const T& y0)
{
    m_xOld = y0;
    m_xOld.zero();
    m_y = y0;
}

template <typename T> const T& Integrator<T>::integrate(const T& x)
{
    // implements the Tustin formula
    iDynTree::toEigen(m_y) = iDynTree::toEigen(m_y) + (iDynTree::toEigen(x) + iDynTree::toEigen(m_xOld)) * (m_dT / 2);
    m_xOld = x;

    return m_y;
}



} // namespace Simulator
} // namespace BipedalLocomotionControllers
