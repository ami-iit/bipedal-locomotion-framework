/**
 * @file VariableFeaibleRegionTask.h
 * @authors Roberto Mauceri
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_TSID_VARIABLE_FEASIBLE_REGION_TASK_H
#define BIPEDAL_LOCOMOTION_BINDINGS_TSID_VARIABLE_FEASIBLE_REGION_TASK_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateVariableFeasibleRegionTask(pybind11::module& module);

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_TSID_VARIABLE_FEASIBLE_REGION_TASK_H
