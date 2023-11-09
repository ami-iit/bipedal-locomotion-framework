/**
 * @file Module.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/RobotDynamicsEstimator/RobotDynamicsEstimator.h>
#include <BipedalLocomotion/bindings/RobotDynamicsEstimator/Module.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotDynamicsEstimator
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "Robot Dynamics Estimator module.";

    CreateRobotDynamicsEstimator(module);
    CreateSubModel(module);
    CreateSubModelCreator(module);
    CreateKinDynWrapper(module);

}
} // namespace RobotDynamicsEstimator
} // namespace bindings
} // namespace BipedalLocomotion
