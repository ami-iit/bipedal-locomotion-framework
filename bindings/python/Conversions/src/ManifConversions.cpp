/**
 * @file ManifConversions.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>

#include <iDynTree/Rotation.h>
#include <iDynTree/Transform.h>

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/bindings/type_caster/swig.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Conversions
{

manif::SE3d toManifPose(::pybind11::object& obj)
{
    const iDynTree::Transform* const cls
        = pybind11::detail::swig_wrapped_pointer_to_pybind<iDynTree::Transform>(obj);

    if (cls == nullptr)
    {
        throw ::pybind11::value_error("Invalid input for the function. Please provide "
                                      "an iDynTree::Transform object.");
    }

    return BipedalLocomotion::Conversions::toManifPose(*cls);
}

manif::SO3d toManifRot(::pybind11::object& obj)
{
    const iDynTree::Rotation* const cls
        = pybind11::detail::swig_wrapped_pointer_to_pybind<iDynTree::Rotation>(obj);

    if (cls == nullptr)
    {
        throw ::pybind11::value_error("Invalid input for the function. Please provide "
                                      "an iDynTree::Rotation object.");
    }

    return BipedalLocomotion::Conversions::toManifRot(*cls);
}

void CreateManifConversions(pybind11::module& module)
{
    namespace py = ::pybind11;

    module
        .def("to_manif_pose",
             py::overload_cast<const Eigen::Matrix<double, 3, 3>&,
                               const Eigen::Matrix<double, 3, 1>&>(
                 &::BipedalLocomotion::Conversions::toManifPose<double>),
             py::arg("rotation"),
             py::arg("translation"))
        .def("to_manif_pose",
             py::overload_cast<::pybind11::object&>(
                 &::BipedalLocomotion::bindings::Conversions::toManifPose),
             py::arg("transform"))
        .def("to_manif_rot",
             py::overload_cast<const Eigen::Matrix<double, 3, 3>&>(
                 &::BipedalLocomotion::Conversions::toManifRot<double>),
             py::arg("rotation"))
        .def("to_manif_rot",
             py::overload_cast<::pybind11::object&>(
                 &::BipedalLocomotion::bindings::Conversions::toManifRot),
             py::arg("rotation"));
}

} // namespace Conversions
} // namespace bindings
} // namespace BipedalLocomotion
