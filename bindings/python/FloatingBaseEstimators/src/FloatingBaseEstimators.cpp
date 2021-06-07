/**
 * @file FloatingBaseEstimators.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>
#include <BipedalLocomotion/FloatingBaseEstimators/ModelComputationsHelper.h>
#include <iDynTree/Model/Model.h>


namespace BipedalLocomotion
{
namespace bindings
{
namespace FloatingBaseEstimators
{

void CreateKinDynComputations(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace iDynTree;
    py::class_<KinDynComputations, std::shared_ptr<KinDynComputations>>(module,
                                                                        "KinDynComputations")
        .def("__repr__",
             [](const KinDynComputations& impl) { return impl.model().toString(); })
        .def("get_nr_of_dofs",
             [](const KinDynComputations& impl) { return impl.getNrOfDegreesOfFreedom(); })
        .def("get_joint_pos",
             [](const KinDynComputations& impl) {
                 Eigen::VectorXd s(impl.getNrOfDegreesOfFreedom());
                 if (!impl.getJointPos(s))
                 {
                     throw py::value_error("Failed to get the joint position.");
                 }
                 return s;
             })
        .def("get_center_of_mass_position",
             [](KinDynComputations& impl) {
                 Eigen::VectorXd pos(3);
                 if (!impl.getCenterOfMassPosition(pos))
                 {
                     throw py::value_error("Failed to get the center of mass position.");
                 }
                 return pos;
             })
        .def("get_center_of_mass_velocity",
             [](KinDynComputations& impl) {
                 Eigen::VectorXd vel(3);
                 if (!impl.getCenterOfMassVelocity(vel))
                 {
                     throw py::value_error("Failed to get the center of mass velocity.");
                 }
                 return vel;
             })
        .def("get_world_transform",
             [](KinDynComputations& impl, std::string name) {
                 Eigen::Matrix<double, 4, 4> world_T_frame;
                 if (!impl.getWorldTransform(name, world_T_frame))
                 {
                     throw py::value_error("Failed to get the trasform for the specified frame.");
                 }
                 return world_T_frame;
             })
        .def(
            "set_joint_pos",
            [](KinDynComputations& impl, Eigen::Ref<const Eigen::VectorXd> s) {
                return impl.setJointPos(iDynTree::make_span(s.data(), s.size()));
            },
            py::arg("s"))
        .def(
            "set_floating_base",
            [](KinDynComputations& impl, const std::string & s) {
                return impl.setFloatingBase(s);
            })
        .def("get_robot_state",
             [](KinDynComputations& impl) {
                 Eigen::Matrix<double, 4, 4> world_T_base;
                 Eigen::VectorXd s(impl.getNrOfDegreesOfFreedom());
                 Eigen::VectorXd base_velocity(6);
                 Eigen::VectorXd s_dot(impl.getNrOfDegreesOfFreedom());
                 Eigen::VectorXd world_gravity(3);
                 if (!impl.getRobotState(world_T_base, s, base_velocity, s_dot, world_gravity))
                 {
                     throw py::value_error("Failed to get the robot state.");
                 }
                 py::dict robot_state;
                 robot_state["world_T_base"] = world_T_base;
                 robot_state["s"] = s;
                 robot_state["base_velocity"] = base_velocity;
                 robot_state["s_dot"] = s_dot;
                 robot_state["world_gravity"] = world_gravity;
                 return robot_state;
             })
        .def("set_robot_state",
             [](KinDynComputations& impl,
                Eigen::Ref<const Eigen::Matrix<double, 4, 4>> world_T_base,
                Eigen::Ref<const Eigen::VectorXd> s,
                Eigen::Ref<const Eigen::VectorXd> base_velocity,
                Eigen::Ref<const Eigen::VectorXd> s_dot,
                Eigen::Ref<const Eigen::VectorXd> world_gravity) {
                 return impl.setRobotState(world_T_base, s, base_velocity, s_dot, world_gravity);
             });
}

void CreateKinDynComputationsDescriptor(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::ParametersHandler;
    using namespace BipedalLocomotion::Estimators;

    py::class_<KinDynComputationsDescriptor>(module, "KinDynComputationsDescriptor")
        .def(py::init())
        .def_readwrite("kindyn", &KinDynComputationsDescriptor::kindyn)
        .def("is_valid", &KinDynComputationsDescriptor::isValid);

    module.def(
        "construct_kindyncomputations_descriptor",
        [](std::shared_ptr<IParametersHandler> handler) -> KinDynComputationsDescriptor {
            return constructKinDynComputationsDescriptor(handler);
        },
        py::arg("handler"));
}

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

    py::class_<Source<Output>>(module, "FloatingBaseEstimatorOutputSource");

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
        .def("advance", &FloatingBaseEstimator::advance)
        .def("reset_estimator",
             py::overload_cast<const InternalState&>(&FloatingBaseEstimator::resetEstimator),
             py::arg("new_state"))
        .def("reset_estimator",
             py::overload_cast<const Eigen::Quaterniond&, const Eigen::Vector3d&>(
                 &FloatingBaseEstimator::resetEstimator),
             py::arg("new_base_orientation"),
             py::arg("new_base_position"))
        .def("get_output", &FloatingBaseEstimator::getOutput)
        .def("is_output_valid", &FloatingBaseEstimator::isOutputValid)
        .def(
            "initialize",
            [](FloatingBaseEstimator& impl,
               std::shared_ptr<IParametersHandler> handler,
               std::shared_ptr<iDynTree::KinDynComputations> kinDyn) -> bool {
                return impl.initialize(handler, kinDyn);
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
