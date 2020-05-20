/**
 * @file FloatingBaseDynamicalSystem.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_FLOATING_BASE_DYNAMICAL_SYSTEM_H
#define BIPEDAL_LOCOMOTION_SYSTEM_FLOATING_BASE_DYNAMICAL_SYSTEM_H

#include <memory>
#include <vector>

#include <BipedalLocomotion/System/DynamicalSystem.h>
#include <BipedalLocomotion/System/ContactWrench.h>

#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>

namespace BipedalLocomotion
{
namespace System
{

/**
 * FloatingBaseDynamicalSystem describes a floating base dynamical system
 */
class FloatingBaseDynamicalSystem : public DynamicalSystem<std::tuple<iDynTree::Vector6,
                                                                      iDynTree::VectorDynSize,
                                                                      iDynTree::Position,
                                                                      iDynTree::Rotation,
                                                                      iDynTree::VectorDynSize>,
                                                           std::tuple<iDynTree::Vector6,
                                                                      iDynTree::VectorDynSize,
                                                                      iDynTree::Vector3,
                                                                      iDynTree::Matrix3x3,
                                                                      iDynTree::VectorDynSize>,
                                                           std::tuple<iDynTree::VectorDynSize,
                                                                      std::vector<ContactWrench>>>
{
    static constexpr size_t m_baseDoFs = 6; /**< Number of degree of freedom associated to the
                                               floating base */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to an existing instance of
                                                               kinDynComputations object */
    std::size_t m_actuatedDoFs{0}; /**< Number of actuated degree of freedom */

    iDynTree::Vector3 m_gravity; /**< Gravity vector */

    iDynTree::MatrixDynSize m_massMatrix; /**< Floating-base mass matrix  */
    iDynTree::FreeFloatingGeneralizedTorques m_generalizedBiasForces; /**< Coriolis and
                                                                         Gravitational term  */
    iDynTree::MatrixDynSize m_jacobianMatrix; /**< Jacobian Matrix  */

    // quantities useful to avoid dynamic allocation in the dynamic allocation in the
    // FloatingBaseDynamicalSystem::dynamics method
    iDynTree::VectorDynSize m_generalizedRobotAcceleration;
    iDynTree::VectorDynSize m_knownCoefficent;

public:
    FloatingBaseDynamicalSystem();

    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    bool dynamics(const std::tuple<const iDynTree::Vector6&,
                                   const iDynTree::VectorDynSize&,
                                   const iDynTree::Position&,
                                   const iDynTree::Rotation&,
                                   const iDynTree::VectorDynSize&>& state,
                  const double& time,
                  const std::tuple<iDynTree::Vector6&,
                                   iDynTree::VectorDynSize&,
                                   iDynTree::Vector3&,
                                   iDynTree::Matrix3x3&,
                                   iDynTree::VectorDynSize&>& stateDerivative) final;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_FLOATING_BASE_DYNAMICAL_SYSTEM_H
