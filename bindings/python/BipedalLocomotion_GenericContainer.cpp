/**
 * @file BipedalLocomotion_GenericContainer.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

#include <BipedalLocomotion/GenericContainer/Vector.h>

#include "BipedalLocomotion_GenericContainer.h"
using namespace BipedalLocomotion::GenericContainer;

namespace BipedalLocomotion
{
namespace bindings
{
namespace
{
namespace py = ::pybind11;
template <typename T> void createVector(pybind11::module& module, const std::string& className)
{
    py::class_<Vector<T>>(module, className.c_str())
        .def(py::init<iDynTree::Span<T>>())
        .def("clone", static_cast<bool (Vector<T>::*)(const Vector<T>&)>(&Vector<T>::clone))
        .def("clone", static_cast<bool (Vector<T>::*)(iDynTree::Span<T>)>(&Vector<T>::clone))
        .def("__len__", &Vector<T>::size)
        .def("resize", &Vector<T>::resize)
        .def("resize_vector", &Vector<T>::resizeVector)
        .def("empty", &Vector<T>::empty);
}

} // namespace

void BipedalLocomotionGenericContainerBindings(pybind11::module& module)
{
    createVector<int>(module, "VectorInt");
    createVector<double>(module, "VectorDouble");
    createVector<std::string>(module, "VectorString");
}

} // namespace bindings
} // namespace BipedalLocomotion
