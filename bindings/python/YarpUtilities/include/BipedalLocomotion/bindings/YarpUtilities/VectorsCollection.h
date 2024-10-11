/**
 * @file VectorsCollection.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_YARP_UTILITES_VECTORS_COLLECTION_H
#define BIPEDAL_LOCOMOTION_BINDINGS_YARP_UTILITES_VECTORS_COLLECTION_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace YarpUtilities
{

void CreateVectorsCollectionServer(pybind11::module& module);
void CreateVectorsCollectionClient(pybind11::module& module);
void CreateVectorsCollectionMetadata(pybind11::module& module);
void CreateVectorsCollection(pybind11::module& module);

} // namespace YarpUtilities
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_YARP_UTILITES_VECTORS_COLLECTION_H
