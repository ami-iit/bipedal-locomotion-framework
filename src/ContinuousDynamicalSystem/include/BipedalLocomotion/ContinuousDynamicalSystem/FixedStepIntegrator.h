/**
 * @file FixedStepIntegrator.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FIXED_STEP_INTEGRATOR_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FIXED_STEP_INTEGRATOR_H

#include <cmath>

#include <BipedalLocomotion/ContinuousDynamicalSystem/Integrator.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * Fixed step integrator base class. Please inherit publicly from this class in order to specify a
 * custom integration method. The custom integration method must define a method called
 * `oneStepIntegration()`
 */
template <class _Derived> class FixedStepIntegrator : public Integrator<_Derived>
{

protected:
    double m_dT{-1}; /**< Fixed step size */

public:
    /**
     * Set the integration step time
     * @param dT integration step time
     */
    bool setIntegrationStep(const double& dT);

    /**
     * Integrate the dynamical system from initialTime to finalTime.
     * @note We assume a constant control input in the interval.
     * @param initialTime initial time of the integration.
     * @param finalTime final time of the integration.
     * @return true in case of success, false otherwise.
     */
    bool integrate(double initialTime, double finalTime);
};

template <class _Derived> bool FixedStepIntegrator<_Derived>::setIntegrationStep(const double& dT)
{
    if (dT <= 0)
    {
        log()->error("[FixedStepIntegrator::setIntegrationStep] The integration must be a strict "
                     "positive number");
        return false;
    }

    m_dT = dT;

    return true;
}

template <class _Derived>
bool FixedStepIntegrator<_Derived>::integrate(double initialTime, double finalTime)
{
    constexpr auto errorPrefix = "[FixedStepIntegrator::integrate]";

    if (this->m_dynamicalSystem == nullptr)
    {
        log()->error("{} Please set the dynamical system before call this function.", errorPrefix);
        return false;
    }

    if (initialTime > finalTime)
    {
        log()->error("{} The final time has to be greater than the initial one.", errorPrefix);
        return false;
    }

    if (m_dT <= 0)
    {
        log()->error("{} Please set the integration step.", errorPrefix);
        return false;
    }

    int iterations = std::ceil((finalTime - initialTime) / m_dT);

    double currentTime = initialTime;
    for (std::size_t i = 0; i < iterations - 1; i++)
    {
        currentTime = initialTime + m_dT * i;

        if (!static_cast<_Derived*>(this)->oneStepIntegration(currentTime, m_dT))
        {
            log()->error("{} Error while integrating at time: {}.", errorPrefix, currentTime);
            return false;
        }
    }

    // Consider last step separately to be sure that the last solution point is in finalTime
    const double dT = finalTime - currentTime;
    if (!static_cast<_Derived*>(this)->oneStepIntegration(currentTime, dT))
    {
        log()->error("{} Error while integrating the last step.", errorPrefix);
        return false;
    }

    return true;
}

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FIXED_STEP_INTEGRATOR_H
