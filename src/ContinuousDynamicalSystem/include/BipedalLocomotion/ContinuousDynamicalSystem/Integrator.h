/**
 * @file Integrator.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_INTEGRATOR_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_INTEGRATOR_H

#include <memory>

#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/impl/traits.h>
#include <BipedalLocomotion/GenericContainer/TemplateHelpers.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * Integrator base class. f$. Please inherit publicly from this class in order to define your custom
 * integrator. Just be sure to call after your class definition #BLF_DEFINE_INTEGRATOR_STRUCTURE()
 */
template <class _Derived> class Integrator
{
public:
    /** DynamicalSystem type */
    using DynamicalSystem = typename internal::traits<_Derived>::DynamicalSystem;

    /** State of the integrator type */
    using State = typename internal::traits<_Derived>::State;

    static_assert(std::is_base_of<
                      BipedalLocomotion::ContinuousDynamicalSystem::DynamicalSystem<DynamicalSystem>,
                      DynamicalSystem>::value,
                  "The integrator template type must be derived from DynamicalSystem.");

protected:
    /** Pointer to a dynamical system*/
    std::shared_ptr<DynamicalSystem> m_dynamicalSystem;

public:
    /**
     * Set the DynamicalSystem to be considered.
     * @note This methods changes the dynamical system only if it was not already set.
     * @param dynamicalSystem Pointer to a dynamical system.
     * @return true in case of success, false otherwise.
     */
    bool setDynamicalSystem(std::shared_ptr<DynamicalSystem> dynamicalSystem);

    /**
     * Get the dynamical system.
     * @return a weak pointer to a dynamical system.
     */
    std::weak_ptr<DynamicalSystem> dynamicalSystem() const;

    /**
     * Retrieve the solution.
     * @return a const reference to the solution.
     */
    const State& getSolution() const;

    /**
     * Integrate the dynamical system from initialTime to finalTime.
     * @note We assume a constant control input in the interval.
     * @param initialTime initial time of the integration.
     * @param finalTime final time of the integration.
     * @return true in case of success, false otherwise.
     */
    bool integrate(double initialTime, double finalTime);
};

template <class _Derived>
bool Integrator<_Derived>::setDynamicalSystem(std::shared_ptr<typename Integrator<_Derived>::DynamicalSystem> dynamicalSystem)
{
    constexpr auto errorPrefix = "[Integrator::setDynamicalSystem]";

    // The dynamical system can be set only once
    if (m_dynamicalSystem != nullptr)
    {
        log()->error("{} The dynamical system has been already set.", errorPrefix);
        return false;
    }

    if (dynamicalSystem == nullptr)
    {
        log()->error("{} The dynamical system passed to the function is corrupted.", errorPrefix);
        return false;
    }

    m_dynamicalSystem = dynamicalSystem;

    return true;
}

template <class _Derived>
std::weak_ptr<typename Integrator<_Derived>::DynamicalSystem> Integrator<_Derived>::dynamicalSystem() const
{
    return m_dynamicalSystem;
}

template <class _Derived>
const typename Integrator<_Derived>::State& Integrator<_Derived>::getSolution() const
{
    return m_dynamicalSystem->getState();
}

template <class _Derived> bool Integrator<_Derived>::integrate(double initialTime, double finalTime)
{
    return static_cast<_Derived*>(this)->integrate(initialTime, finalTime);
}

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_INTEGRATOR_H
