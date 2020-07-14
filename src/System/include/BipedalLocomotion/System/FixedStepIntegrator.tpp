/**
 * @file FixedStepIntegrator.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_FIXED_STEP_INTEGRATOR_TPP
#define BIPEDAL_LOCOMOTION_SYSTEM_FIXED_STEP_INTEGRATOR_TPP

#include <cmath>
#include <memory>
#include <tuple>

#include <BipedalLocomotion/System/FixedStepIntegrator.h>

namespace BipedalLocomotion
{
namespace System
{
template <typename DynamicalSystemDerived>
bool FixedStepIntegrator<DynamicalSystemDerived>::integrate(double initialTime, double finalTime)
{
    if (this->m_dynamicalSystem == nullptr)
    {
        std::cerr << "[FixedStepIntegrator::integrate] Please set the dynamical system before call "
                     "this function."
                  << std::endl;
        return false;
    }

    if (initialTime > finalTime)
    {
        std::cerr << "[FixedStepIntegrator::integrate] The final time has to be greater than the "
                     "initial one."
                  << std::endl;
        return false;
    }

    if (m_dT <= 0)
    {
        std::cerr << "[FixedStepIntegrator::integrate] The sampling time must be a strictly "
                     "positive number."
                  << std::endl;
        return false;
    }

    int iterations = std::ceil((finalTime - initialTime) / m_dT);

    double currentTime = initialTime;
    for (std::size_t i = 0; i < iterations - 1; i++)
    {
        currentTime = initialTime + m_dT * i;

        if (!oneStepIntegration(currentTime, m_dT))
        {
            std::cerr << "[FixedStepIntegrator::integrate] Error while integrating at time: "
                      << currentTime << "." << std::endl;
            return false;
        }
    }

    // Consider last step separately to be sure that the last solution point is in finalTime
    double dT = finalTime - currentTime;
    if (!oneStepIntegration(currentTime, dT))
    {
        std::cerr << "[FixedStepIntegrator::integrate] Error while integrating the last step."
                  << std::endl;
        return false;
    }
    return true;
}

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_FIXED_STEP_INTEGRATOR_TPP
