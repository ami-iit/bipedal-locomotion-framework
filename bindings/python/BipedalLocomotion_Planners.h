/**
 * @file BipedalLocomotion_Planners.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PYTHON_BINDINGS_PLANNERS_H
#define BIPEDAL_LOCOMOTION_PYTHON_BINDINGS_PLANNERS_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{

void BipedalLocomotionPlannersBindings(pybind11::module& module);

} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PYTHON_BINDINGS_PLANNERS_H
