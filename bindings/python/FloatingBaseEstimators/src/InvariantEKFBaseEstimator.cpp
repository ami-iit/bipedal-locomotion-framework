/**
 * @file InvariantEKFBaseEstimator.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/attr.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>
#include <BipedalLocomotion/FloatingBaseEstimators/InvariantEKFBaseEstimator.h>

#include <BipedalLocomotion/bindings/FloatingBaseEstimators/InvariantEKFBaseEstimator.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace FloatingBaseEstimators
{

void CreateInvariantEKFBaseEstimator(pybind11::module& module)
{
    namespace py = ::pybind11;
    namespace BLE = ::BipedalLocomotion::Estimators;

    py::class_<BLE::InvariantEKFBaseEstimator, //
               BLE::FloatingBaseEstimator>(module, "InvariantEKFBaseEstimator")
        .def(py::init())
        .def("toggle_bias_estimation",
             &BLE::InvariantEKFBaseEstimator::toggleBiasEstimation,
             py::arg("flag"))
        .def("toggle_ekf_update", &BLE::InvariantEKFBaseEstimator::toggleEKFUpdate, py::arg("flag"))
        .def("reset_estimator",
             py::overload_cast<const BLE::FloatingBaseEstimators::InternalState&,
                               const BLE::FloatingBaseEstimators::StateStdDev&>(
                 &BLE::InvariantEKFBaseEstimator::resetEstimator))
        .def("reset_estimator",
             py::overload_cast<const BLE::FloatingBaseEstimators::InternalState&,
                               const BLE::FloatingBaseEstimators::StateStdDev&,
                               const BLE::FloatingBaseEstimators::SensorsStdDev&>(
                 &BLE::InvariantEKFBaseEstimator::resetEstimator))
        .def("reset_estimator",
             py::overload_cast<const BLE::FloatingBaseEstimators::InternalState&>(
                 &BLE::InvariantEKFBaseEstimator::resetEstimator),
             py::arg("new_state"))
        .def(
            "reset_estimator",
            [](BLE::InvariantEKFBaseEstimator& impl,
               const manif::SO3d& newBaseOrientation,
               const Eigen::Vector3d& newBasePosition) -> bool {
                return impl.resetEstimator(newBaseOrientation.quat(), newBasePosition);
            },
            py::arg("new_base_orientation"),
            py::arg("new_base_position"));
}

} // namespace FloatingBaseEstimators
} // namespace bindings
} // namespace BipedalLocomotion
