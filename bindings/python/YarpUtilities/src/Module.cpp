/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/YarpUtilities/VectorsCollection.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace YarpUtilities
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "YarpUtilities module.";

    CreateVectorsCollectionServer(module);
    CreateVectorsCollectionClient(module);
    CreateVectorsCollectionMetadata(module);
    CreateVectorsCollection(module);
}
} // namespace YarpUtilities
} // namespace bindings
} // namespace BipedalLocomotion
