/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/ML/MANN.h>
#include <BipedalLocomotion/bindings/ML/MANNAutoregressive.h>
#include <BipedalLocomotion/bindings/ML/MANNTrajectoryGenerator.h>
#include <BipedalLocomotion/bindings/ML/MANNAutoregressiveInputBuilder.h>


namespace BipedalLocomotion
{
namespace bindings
{
namespace ML
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "ML module.";

    CreateMANN(module);
    CreateMANNAutoregressive(module);
    CreateMANNTrajectoryGenerator(module);
    CreateMANNAutoregressiveInputBuilder(module);
}
} // namespace ML
} // namespace bindings
} // namespace BipedalLocomotion
