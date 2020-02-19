/**
 * @file Integrator.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_INTEGRATOR_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_INTEGRATOR_H

#include <cstddef>

namespace BipedalLocomotionControllers
{
namespace Simulator
{

template <class T> class Integrator
{
private:
    T m_y;
    T m_xOld;
    double m_dT;



 public:
    Integrator(const double dT, const T& y0);
    Integrator(const double dT);

    void reset(const T& y0);

    const T& integrate(const T& x);

    const T& get() const
    {
        return m_y;
    }
};

} // namespace Simulator
} // namespace BipedalLocomotionControllers

#include "Integrator.tpp"

#endif //
