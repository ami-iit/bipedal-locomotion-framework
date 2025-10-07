/**
 * @file TimeProfiler.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_TIME_PROFILER_H
#define BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_TIME_PROFILER_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{

void CreateTimeProfiler(pybind11::module& module);


} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_VARIABLES_HANDLER_H
