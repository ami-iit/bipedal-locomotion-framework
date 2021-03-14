/**
 * @file QuinticSpline.cpp
 * @authors Diego Ferigo
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <manif/SE3.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/bindings/manif/BaseTypes.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace manif
{

std::string toString(const ::manif::SE3d& se3)
{
    // Custom formatter of Eigen vectors
    const Eigen::IOFormat FormatEigenVector //
        (Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

    const auto& position = se3.coeffs().segment<3>(0);
    const auto& quaternion = se3.coeffs().segment<4>(3);

    std::stringstream repr;

    repr << "SE3 (position = " << position.format(FormatEigenVector)
         << ", quaternion = " << quaternion.format(FormatEigenVector) << ")";

    return repr.str();
}

void CreateBaseTypes(pybind11::module& module)
{
    namespace py = ::pybind11;

    py::class_<::manif::SE3d>(module, "SE3")
        .def(py::init([]() { //
            return ::manif::SE3d(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
        }))
        .def(py::init([](const Eigen::Vector3d& pos, const Eigen::Vector4d& quat) {
                 return ::manif::SE3d(pos, quat);
             }),
             py::arg("position"),
             py::arg("quaternion"))
        .def_property(
            "position",
            [](const ::manif::SE3d& se3) -> Eigen::Vector3d { return se3.coeffs().segment<3>(0); },
            [](::manif::SE3d& se3, const Eigen::Vector3d& position) {
                se3 = ::manif::SE3d({position, se3.coeffs().segment<4>(3)});
            })
        .def_property(
            "quaternion",
            [](const ::manif::SE3d& se3) -> Eigen::Vector4d { return se3.coeffs().segment<4>(3); },
            [](::manif::SE3d& se3, const Eigen::Vector4d& quaternion) {
                se3 = ::manif::SE3d({se3.coeffs().segment<3>(0), quaternion});
            })
        .def("__repr__", &toString)
        .def(
            "__eq__",
            [](::manif::SE3d& lhs, ::manif::SE3d& rhs) -> bool { return lhs == rhs; },
            py::is_operator());
}

} // namespace manif
} // namespace bindings
} // namespace BipedalLocomotion
