/**
 * @file Integrator.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_INTEGRATOR_H
#define BIPEDAL_LOCOMOTION_SYSTEM_INTEGRATOR_H

#include <memory>
#include <tuple>

#include <BipedalLocomotion/GenericContainer/TemplateHelpers.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/DynamicalSystem.h>

namespace BipedalLocomotion
{
namespace System
{

/**
 * Integrator base class. Please inherit publicly from this class in order to specify a custom
 * integration method.
 * @tparam DynamicalSystemDerived a class derived from DynamicalSystem
 */
template <typename DynamicalSystemDerived> class Integrator
{
    static_assert(
        std::is_base_of<DynamicalSystem<typename DynamicalSystemDerived::StateType,
                                        typename DynamicalSystemDerived::StateDerivativeType,
                                        typename DynamicalSystemDerived::InputType>,
                        DynamicalSystemDerived>::value,
        "The integrator template type has to be derived from DynamicalSystem.");

protected:

    /** Pointer to a dynamical system*/
    std::shared_ptr<DynamicalSystemDerived> m_dynamicalSystem;

    /** Solution of the integrator */
    typename DynamicalSystemDerived::StateType m_solution;

public:

    /**
     * Set the DynamicalSystem to be considered.
     * @note This methods changes the dynamical system only if it was not already set.
     * @param dynamicalSystem Pointer to a dynamical system.
     * @return true in case of success, false otherwise.
     */
    bool setDynamicalSystem(std::shared_ptr<DynamicalSystemDerived> dynamicalSystem);

    /**
     * Get the dynamical system.
     * @return a weak pointer to a dynamical system.
     */
    const std::weak_ptr<DynamicalSystemDerived> dynamicalSystem () const;

    /**
     * Retrieve the solution.
     * @return a const reference to the solution.
     */
    const typename DynamicalSystemDerived::StateType& getSolution() const;

    /**
     * Integrate the dynamical system from initialTime to finalTime.
     * @note We assume a constant control input in the interval.
     * @param initialTime initial time of the integration.
     * @param finalTime final time of the integration.
     * @return true in case of success, false otherwise.
     */
    virtual bool integrate(double initialTime, double finalTime) = 0;

    ~Integrator() = default;
};

} // namespace System
} // namespace BipedalLocomotion


#include <BipedalLocomotion/System/Integrator.tpp>

#endif // BIPEDAL_LOCOMOTION_SYSTEM_DYNAMICAL_SYSTEM_H
