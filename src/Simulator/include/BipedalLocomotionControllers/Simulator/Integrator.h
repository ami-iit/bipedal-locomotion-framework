/**
 * @file Integrator.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_INTEGRATOR_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_INTEGRATOR_H

namespace BipedalLocomotionControllers
{
namespace Simulator
{

/**
 * A class for defining an integrator based on Tustin formula.
 */
template <class T> class Integrator
{
    T m_y; /**< Integrated signal */
    T m_xOld; /**< Previous value of the input vector that will be integrated */
    double m_dT; /**< Sampling Time in second */
public:
    /**
     * Creates a new Integrator
     * @param dT sampling time
     * @param y0 the initial new value
     */
    Integrator(const double dT, const T& y0);

    /**
     * Creates a new Integrator
     * @param dT sampling time
     */
    Integrator(const double dT);

    /**
     * Resets the internal state and sets the output vector to the given value.
     * @param y0 the initial new value
     */
    void reset(const T& y0);

    /**
     * Executes one-step integration of input vector.
     * @param x is the input vector that will be integrated
     * @return the integrated signal
     */
    const T& integrate(const T& x);

    const T& get() const;
};

} // namespace Simulator
} // namespace BipedalLocomotionControllers

#include "Integrator.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_INTEGRATOR_H
