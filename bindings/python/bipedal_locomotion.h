/**
 * @file bipedal_locomotion.h
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>

namespace BipedalLocomotion::bindings
{
// QuinticSpline.cpp
void CreateQuinticSpline(pybind11::module& module);

// ParametersHandler.cpp
void CreateIParameterHandler(pybind11::module& module);
void CreateStdParameterHandler(pybind11::module& module);
} // namespace BipedalLocomotion::bindings
