/**
 * @file Constants.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_MATH_WRENCH_H
#define BIPEDAL_LOCOMOTION_BINDINGS_MATH_WRENCH_H

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <BipedalLocomotion/Math/Wrench.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Math
{

template <typename Scalar> struct WrenchTrampoline
{
    BipedalLocomotion::Math::Wrench<Scalar> wrench;
};

template <typename Scalar> void CreateWrench(pybind11::module& module, const std::string& suffix)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Math;

    py::class_<WrenchTrampoline<Scalar>>(module, ("Wrench" + suffix).c_str())
        .def(py::init<>())
        .def(py::init([](const Eigen::Matrix<Scalar, 6, 1>& vector) -> WrenchTrampoline<Scalar> {
            WrenchTrampoline<Scalar> tmp;
            tmp.wrench = vector;
            return tmp;
        }))
        .def("get_local_cop",
             [](const WrenchTrampoline<Scalar>& impl) -> Eigen::Matrix<Scalar, 3, 1> {
                 return impl.wrench.getLocalCoP();
             })
        .def_property(
            "force",
            [](const WrenchTrampoline<Scalar>& impl)
                -> Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>> { return impl.wrench.force(); },
            [](WrenchTrampoline<Scalar>& impl, const Eigen::Matrix<Scalar, 3, 1>& force) {
                impl.wrench.force() = force;
            })
        .def_property(
            "torque",
            [](const WrenchTrampoline<Scalar>& impl)
                -> Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>> { return impl.wrench.torque(); },
            [](WrenchTrampoline<Scalar>& impl, const Eigen::Matrix<Scalar, 3, 1>& torque) {
                impl.wrench.torque() = torque;
            });
}
} // namespace Math
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_MATH_WRENCH_H
