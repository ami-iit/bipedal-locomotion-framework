/**
 * @file Integrator.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_INTEGRATOR_TPP
#define BIPEDAL_LOCOMOTION_SYSTEM_INTEGRATOR_TPP

#include <BipedalLocomotion/System/Integrator.h>

namespace BipedalLocomotion
{
namespace System
{

template <typename DynamicalSystemDerived>
bool Integrator<DynamicalSystemDerived>::setDynamicalSystem(std::shared_ptr<DynamicalSystemDerived> dynamicalSystem)
{
    // The dynamical system can be set only once
    if (m_dynamicalSystem != nullptr)
    {
        std::cerr << "[Integrator::setDynamicalSystem] The dynamical system has been already set."
                  << std::endl;
        return false;
    }

    if (dynamicalSystem == nullptr)
    {
        std::cerr << "[Integrator::setDynamicalSystem] The dynamical system passed to the function "
                     "is corrupted."
                  << std::endl;
        return false;
    }

    m_dynamicalSystem = dynamicalSystem;

    // set the initial state
    m_solution = m_dynamicalSystem->getInitialState();

    return true;
}

template <typename DynamicalSystemDerived>
const std::weak_ptr<DynamicalSystemDerived>
Integrator<DynamicalSystemDerived>::dynamicalSystem() const
{
    return m_dynamicalSystem;
}

template <typename DynamicalSystemDerived>
const typename DynamicalSystemDerived::StateType&
Integrator<DynamicalSystemDerived>::getSolution() const
{
    return m_solution;
}

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_DYNAMICAL_SYSTEM_H
