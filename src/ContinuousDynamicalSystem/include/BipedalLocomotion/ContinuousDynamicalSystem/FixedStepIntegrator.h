/**
 * @file FixedStepIntegrator.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FIXED_STEP_INTEGRATOR_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FIXED_STEP_INTEGRATOR_H

#include <chrono>
#include <cmath>

#include <BipedalLocomotion/ContinuousDynamicalSystem/Integrator.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{
template <typename _Derived> class FixedStepIntegrator;
}
}

BLF_DEFINE_INTEGRATOR_STRUCTURE(FixedStepIntegrator, _Derived)


namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * Fixed step integrator base class. Please inherit publicly from this class in order to specify a
 * custom integration method. The custom integration method must define a method called
 * `oneStepIntegration()`
 */
template <class _Derived>
class FixedStepIntegrator : public Integrator<FixedStepIntegrator<_Derived>>
{
public:
    using DynamicalSystem = typename internal::traits<FixedStepIntegrator<_Derived>>::DynamicalSystem;
    using State = typename internal::traits<FixedStepIntegrator<_Derived>>::State;
    using StateDerivative = typename internal::traits<FixedStepIntegrator<_Derived>>::StateDerivative;

protected:
    std::chrono::nanoseconds m_dT{std::chrono::nanoseconds::min()}; /**< Fixed step size */

public:
    /**
     * Set the integration step time
     * @param dT integration step time
     */
    bool setIntegrationStep(const std::chrono::nanoseconds& dT);

    /**
     * get the integration step time
     * @return the integration step time
     */
     const std::chrono::nanoseconds& getIntegrationStep() const;

    /**
     * Integrate the dynamical system from initialTime to finalTime.
     * @note We assume a constant control input in the interval.
     * @param initialTime initial time of the integration.
     * @param finalTime final time of the integration.
     * @return true in case of success, false otherwise.
     */
    bool integrate(const std::chrono::nanoseconds& initialTime,
                   const std::chrono::nanoseconds& finalTime);
};

template <class _Derived> bool FixedStepIntegrator<_Derived>::setIntegrationStep(const std::chrono::nanoseconds& dT)
{
    if (dT <= std::chrono::nanoseconds::zero())
    {
        log()->error("[FixedStepIntegrator::setIntegrationStep] The integration must be a strict "
                     "positive number");
        return false;
    }

    m_dT = dT;

    return true;
}

template <class _Derived>
const std::chrono::nanoseconds& FixedStepIntegrator<_Derived>::getIntegrationStep() const
{
    return m_dT;
}

template <class _Derived>
bool FixedStepIntegrator<_Derived>::integrate(const std::chrono::nanoseconds& initialTime,
                                              const std::chrono::nanoseconds& finalTime)
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

    if (m_dT <= std::chrono::nanoseconds::zero())
    {
        log()->error("{} Please set the integration step.", errorPrefix);
        return false;
    }

    const std::size_t iterations = (finalTime - initialTime) / m_dT;
    std::chrono::nanoseconds currentTime = initialTime;
    for (std::size_t i = 0; i < iterations - 1; i++)
    {
        // advance the current time
        currentTime += m_dT;

        if (!static_cast<_Derived*>(this)->oneStepIntegration(currentTime, m_dT))
        {
            log()->error("{} Error while integrating at time: {}.", errorPrefix, currentTime);
            return false;
        }
    }

    // Consider last step separately to be sure that the last solution point is in finalTime
    const std::chrono::nanoseconds dT = finalTime - currentTime;
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
