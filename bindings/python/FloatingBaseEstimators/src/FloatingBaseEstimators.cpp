/**
 * @file FloatingBaseEstimators.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>
#include <BipedalLocomotion/Math/Constants.h>

#include <BipedalLocomotion/bindings/System/Advanceable.h>
#include <BipedalLocomotion/bindings/type_caster/swig.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace FloatingBaseEstimators
{

void CreateFloatingBaseEstimator(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;
    using namespace BipedalLocomotion::ParametersHandler;
    using namespace BipedalLocomotion::Contacts;
    using namespace BipedalLocomotion::Estimators;
    using namespace BipedalLocomotion::Estimators::FloatingBaseEstimators;

    py::class_<InternalState>(module, "FloatingBaseEstimatorInternalState")
        .def(py::init())
        .def_readwrite("imu_orientation", &InternalState::imuOrientation)
        .def_readwrite("imu_position", &InternalState::imuPosition)
        .def_readwrite("imu_linear_velocity", &InternalState::imuLinearVelocity)
        .def_readwrite("imu_angular_velocity", &InternalState::imuAngularVelocity)
        .def_readwrite("support_frame_data", &InternalState::supportFrameData)
        .def_readwrite("accelerometer_bias", &InternalState::accelerometerBias)
        .def_readwrite("gyroscope_bias", &InternalState::gyroscopeBias)
        .def_readwrite("l_contact_frame_orientation", &InternalState::lContactFrameOrientation)
        .def_readwrite("l_contact_frame_position", &InternalState::lContactFramePosition)
        .def_readwrite("r_contact_frame_orientation", &InternalState::rContactFrameOrientation)
        .def_readwrite("r_contact_frame_position", &InternalState::rContactFramePosition);

    py::class_<StateStdDev>(module, "FloatingBaseEstimatorStateStdDev")
        .def(py::init())
        .def_readwrite("imu_orientation", &StateStdDev::imuOrientation)
        .def_readwrite("imu_position", &StateStdDev::imuPosition)
        .def_readwrite("imu_linear_velocity", &StateStdDev::imuLinearVelocity)
        .def_readwrite("accelerometer_bias", &StateStdDev::accelerometerBias)
        .def_readwrite("gyroscope_bias", &StateStdDev::gyroscopeBias)
        .def_readwrite("l_contact_frame_orientation", &StateStdDev::lContactFrameOrientation)
        .def_readwrite("l_contact_frame_position", &StateStdDev::lContactFramePosition)
        .def_readwrite("r_contact_frame_orientation", &StateStdDev::rContactFrameOrientation)
        .def_readwrite("r_contact_frame_position", &StateStdDev::rContactFramePosition);

    py::class_<Output>(module, "FloatingBaseEstimatorOutput")
        .def(py::init())
        .def_readwrite("state", &Output::state)
        .def_readwrite("state_std_dev", &Output::stateStdDev)
        .def_readwrite("base_pose", &Output::basePose)
        .def_readwrite("base_twist", &Output::baseTwist);

    py::class_<Measurements>(module, "FloatingBaseEstimatorMeasurements")
        .def(py::init())
        .def_readwrite("acc", &Measurements::acc)
        .def_readwrite("gyro", &Measurements::gyro)
        .def_readwrite("encoders", &Measurements::encoders)
        .def_readwrite("encoder_speeds", &Measurements::encodersSpeed)
        .def_readwrite("stamped_contact_status", &Measurements::stampedContactsStatus)
        .def_readwrite("lf_in_contact", &Measurements::lfInContact)
        .def_readwrite("rf_in_contact", &Measurements::rfInContact);

    py::class_<Options>(module, "FloatingBaseEstimatorOptions")
        .def(py::init())
        .def_readwrite("imu_bias_estimation_enabled", &Options::imuBiasEstimationEnabled)
        .def_readwrite("static_imu_bias_initialization_enabled",
                       &Options::staticImuBiasInitializationEnabled)
        .def_readwrite("nr_samples_for_imu_bias_initialization",
                       &Options::nrSamplesForImuBiasInitialization)
        .def_readwrite("ekf_update_enabled", &Options::ekfUpdateEnabled)
        .def_readwrite("acceleration_due_to_gravity", &Options::accelerationDueToGravity);

    py::class_<SensorsStdDev>(module, "FloatingBaseEstimatorSensorsStdDev")
        .def(py::init())
        .def_readwrite("accelerometer_noise", &SensorsStdDev::accelerometerNoise)
        .def_readwrite("gyroscope_noise", &SensorsStdDev::gyroscopeNoise)
        .def_readwrite("accelerometer_bias_noise", &SensorsStdDev::accelerometerBiasNoise)
        .def_readwrite("gyroscope_bias_noise", &SensorsStdDev::gyroscopeBiasNoise)
        .def_readwrite("contact_foot_lin_vel_noise", &SensorsStdDev::contactFootLinvelNoise)
        .def_readwrite("contact_foot_ang_vel_noise", &SensorsStdDev::contactFootAngvelNoise)
        .def_readwrite("swing_foot_lin_vel_noise", &SensorsStdDev::swingFootLinvelNoise)
        .def_readwrite("swing_foot_ang_vel_noise", &SensorsStdDev::swingFootAngvelNoise)
        .def_readwrite("forward_kinematics_noise", &SensorsStdDev::forwardKinematicsNoise)
        .def_readwrite("encoders_noise", &SensorsStdDev::encodersNoise);

    BipedalLocomotion::bindings::System::CreateSource<Output>(module,
                                                              "FloatingBaseEstimatorOutput");

    py::class_<FloatingBaseEstimator, Source<Output>> floatingBaseEstimator( //
        module,
        "FloatingBaseEstimator");

    floatingBaseEstimator.def(py::init())
        .def("set_imu_measurement",
             &FloatingBaseEstimator::setIMUMeasurement,
             py::arg("acc_meas"),
             py::arg("gyro_meas"))
        .def("set_contact_status",
             &FloatingBaseEstimator::setContactStatus,
             py::arg("name"),
             py::arg("contact_status"),
             py::arg("switch_time"),
             py::arg("time_now"))
        .def("set_kinematics",
             &FloatingBaseEstimator::setKinematics,
             py::arg("encoders"),
             py::arg("encoder_speeds"))
        .def("reset_estimator",
             py::overload_cast<const InternalState&>(&FloatingBaseEstimator::resetEstimator),
             py::arg("new_state"))
        .def("reset_estimator",
             py::overload_cast<const Eigen::Quaterniond&, const Eigen::Vector3d&>(
                 &FloatingBaseEstimator::resetEstimator),
             py::arg("new_base_orientation"),
             py::arg("new_base_position"))
        .def(
            "initialize",
            [](FloatingBaseEstimator& impl,
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
            py::arg("kindyn"));

    // at the moment at an interface level, this nested class need not be exposed
    // but in case, we want to use the FloatingBaseEstimator as base class
    // for python based implementations, maybe it could be useful
    py::class_<FloatingBaseEstimator::ModelComputations>(floatingBaseEstimator,
                                                         "FloatingBaseEstimatorModelComputations")
        .def(py::init());
}

} // namespace FloatingBaseEstimators
} // namespace bindings
} // namespace BipedalLocomotion
