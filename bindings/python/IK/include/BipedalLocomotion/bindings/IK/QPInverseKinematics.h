/**
 * @file QPInverseKinematics.h
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_IK_QP_INVERSE_KINEMATICS_H
#define BIPEDAL_LOCOMOTION_BINDINGS_IK_QP_INVERSE_KINEMATICS_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateQPInverseKinematics(pybind11::module& module);

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_IK_QP_INVERSE_KINEMATICS_H
