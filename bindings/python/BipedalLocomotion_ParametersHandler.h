/**
 * @file BipedalLocomotion_ParameterHandler.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PYTHON_BINDINGS_PARAMETER_HANDLER_H
#define BIPEDAL_LOCOMOTION_PYTHON_BINDINGS_PARAMETER_HANDLER_H


#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{

void BipedalLocomotionParametersHandlerBindings(pybind11::module& module);

} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PYTHON_BINDINGS_PARAMETER_HANDLER_H
