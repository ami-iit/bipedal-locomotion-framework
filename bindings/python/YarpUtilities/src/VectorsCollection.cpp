/**
 * @file VectorsCollection.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/YarpUtilities/VectorsCollection.h>

#include <BipedalLocomotion/bindings/YarpUtilities/VectorsCollection.h>
#include <BipedalLocomotion/bindings/YarpUtilities/BufferedPort.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace YarpUtilities
{
void CreateVectorsCollection(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace ::BipedalLocomotion::YarpUtilities;

    py::class_<VectorsCollection>(module, "VectorsCollection")
        .def(py::init())
        .def_readwrite("vectors", &VectorsCollection::vectors)
        .def("__repr__", &VectorsCollection::toString)
        .def("to_string", &VectorsCollection::toString);

    CreateBufferedPort<VectorsCollection>(module, "BufferedPortVectorsCollection");
}
} // namespace YarpUtilities
} // namespace bindings
} // namespace BipedalLocomotion
