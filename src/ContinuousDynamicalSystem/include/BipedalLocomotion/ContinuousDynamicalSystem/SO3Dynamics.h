/**
 * @file SO3Dynamics.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_SO3_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_SO3_DYNAMICS_H

#include <memory>
#include <vector>

#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/LieGroupDynamics.h>
#include <BipedalLocomotion/GenericContainer/NamedTuple.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

#include <manif/SO3.h>

#include <Eigen/Dense>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * SO3Dynamics describes the dynamics of a SO(3).
 * In details, given an element of SO(3) \f$R \in SO(3)\f$ and a vector of its Lie Algebra
 * \f$ \omega^\wedge \in \mathrm{so}(3)\f$, SO3Dynamics implements
 * \f[
 * \dot{R} = \omega^\wedge  R
 * \f]
 * where \f$\dot{R} \in T_R SO(3)\f$ with \f$T_R SO(3)\f$ is the Tangent space of \f$SO(3)\f$ at \f$R\f$.

 * The SO3Dynamics inherits from a generic DynamicalSystem where the State is
 * described by a BipedalLocomotion::GenericContainer::named_tuple
 * |    Name    |        Type       |         Description        |
 * |:----------:|:-----------------:|:--------------------------:|
 * | `LieGroup` |   `manif::SO3d`   |      The rotation matrix   |
 *
 * The `StateDerivative` is described by a BipedalLocomotion::GenericContainer::named_tuple
 * |    Name    |           Type         |            Description                |
 * |:----------:|:----------------------:|:-------------------------------------:|
 * | `Tangent`  | `manif::SO3d::Tangent` |  Angular velocity in inertial frame   |
 *
 * The `Input` is described by a BipedalLocomotion::GenericContainer::named_tuple
 * |    Name    |           Type         |            Description                |
 * |:----------:|:----------------------:|:-------------------------------------:|
 * | `Tangent`  | `manif::SO3d::Tangent` |  Angular velocity in inertial frame   |
 */
using SO3Dynamics = LieGroupDynamics<manif::SO3d>;

}
}

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_SO3_DYNAMICS_H
