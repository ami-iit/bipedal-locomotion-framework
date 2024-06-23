/**
 * @file LeggedOdometry.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>
#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>

#include <BipedalLocomotion/bindings/FloatingBaseEstimators/LeggedOdometry.h>
#include <BipedalLocomotion/bindings/type_caster/swig.h>

#include <iDynTree/KinDynComputations.h>

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
               py::object& obj) -> bool {
                std::shared_ptr<iDynTree::KinDynComputations>* kinDynPtr
                    = pybind11::detail::swig_wrapped_pointer_to_pybind<
                        std::shared_ptr<iDynTree::KinDynComputations>>(obj);

                if (kinDynPtr == nullptr)
                {
                    throw ::pybind11::value_error("Invalid input for the function. Please provide "
                                                  "an iDynTree::KinDynComputations object.");
                }

                return impl.initialize(handler, *kinDynPtr);
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
             py::overload_cast<const InternalState&>(&FloatingBaseEstimator::resetEstimator),
             py::arg("new_state"))
        .def("reset_estimator",
             py::overload_cast<const Eigen::Quaterniond&, const Eigen::Vector3d&>(
                 &FloatingBaseEstimator::resetEstimator),
             py::arg("new_base_orientation"),
             py::arg("new_base_position"))
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
                               const Eigen::Vector4d&,
                               const Eigen::Vector3d&>(&LeggedOdometry::changeFixedFrame),
             py::arg("frame_index"),
             py::arg("frame_orientation_in_world"),
             py::arg("frame_position_in_world"))
        .def("get_fixed_frame_index", &LeggedOdometry::getFixedFrameIdx)
        .def("get_output", &LeggedOdometry::getOutput)
        .def("is_output_valid", &LeggedOdometry::isOutputValid);
}

} // namespace FloatingBaseEstimators
} // namespace bindings
} // namespace BipedalLocomotion
