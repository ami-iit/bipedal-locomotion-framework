/**
 * @file LeggedOdometry.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>
#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>

#include <BipedalLocomotion/bindings/FloatingBaseEstimators/LeggedOdometry.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace FloatingBaseEstimators
{

void CreateLeggedOdometry(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;
    using namespace BipedalLocomotion::ParametersHandler;
    using namespace BipedalLocomotion::Contacts;
    using namespace BipedalLocomotion::Estimators;
    using namespace BipedalLocomotion::Estimators::FloatingBaseEstimators;

    py::class_<LeggedOdometry, FloatingBaseEstimator>(module, "LeggedOdometry")
        .def(py::init())
        .def(
            "initialize",
            [](LeggedOdometry& impl,
               std::shared_ptr<IParametersHandler> handler,
               std::shared_ptr<iDynTree::KinDynComputations> kinDyn) -> bool {
                return impl.initialize(handler, kinDyn);
            },
            py::arg("handler"),
            py::arg("kindyn"))
        .def("set_imu_measurement",
             &LeggedOdometry::setIMUMeasurement,
             py::arg("acc_meas"),
             py::arg("gyro_meas"))
        .def("set_contact_status",
             &LeggedOdometry::setContactStatus,
             py::arg("name"),
             py::arg("contact_status"),
             py::arg("switch_time"),
             py::arg("time_now"))
        .def("set_kinematics",
             &LeggedOdometry::setKinematics,
             py::arg("encoders"),
             py::arg("encoder_speeds"))
        .def("advance", &LeggedOdometry::advance)
        .def("reset_estimator",
             py::overload_cast<const InternalState&>(&LeggedOdometry::resetEstimator),
             py::arg("new_state"))
        .def("reset_estimator",
             py::overload_cast<const Eigen::Quaterniond&, const Eigen::Vector3d&>(
                 &LeggedOdometry::resetEstimator),
             py::arg("new_imu_orientation"),
             py::arg("new_imu_position"))
        .def("reset_estimator", py::overload_cast<>(&LeggedOdometry::resetEstimator))
        .def("reset_estimator",
             py::overload_cast<const std::string&, const Eigen::Quaterniond&, const Eigen::Vector3d&>(
                 &LeggedOdometry::resetEstimator),
             py::arg("ref_frame_for_world"),
             py::arg("world_orientation_in_ref_frame"),
             py::arg("world_position_in_ref_frame"))
        .def("change_fixed_frame",
             py::overload_cast<const std::ptrdiff_t&>(&LeggedOdometry::changeFixedFrame),
             py::arg("frame_index"))
        .def("change_fixed_frame",
             py::overload_cast<const std::ptrdiff_t&,
                               const Eigen::Quaterniond&,
                               const Eigen::Vector3d&>(&LeggedOdometry::changeFixedFrame),
             py::arg("frame_index"),
             py::arg("frame_orientation_in_world"),
             py::arg("frame_position_in_world"))
        .def("get_fixed_frame_index", &LeggedOdometry::getFixedFrameIdx)
        .def("get", &LeggedOdometry::get)
        .def("is_valid", &LeggedOdometry::isValid);
}

} // namespace FloatingBaseEstimators
} // namespace bindings
} // namespace BipedalLocomotion
