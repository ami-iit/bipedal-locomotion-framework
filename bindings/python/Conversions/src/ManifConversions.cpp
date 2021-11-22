/**
 * @file ManifConversions.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>

#include <BipedalLocomotion/Conversions/ManifConversions.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Conversions
{

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
        .def("to_manif_rot",
             py::overload_cast<const Eigen::Matrix<double, 3, 3>&>(
                 &::BipedalLocomotion::Conversions::toManifRot<double>),
             py::arg("rotation"));
}

} // namespace Conversions
} // namespace bindings
} // namespace BipedalLocomotion
