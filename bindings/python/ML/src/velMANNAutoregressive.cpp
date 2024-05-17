/**
 * @file velMANNAutoregressive.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <iDynTree/Model/Model.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ML/velMANNAutoregressive.h>
#include <BipedalLocomotion/bindings/ML/velMANNAutoregressive.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>
#include <BipedalLocomotion/bindings/type_caster/swig.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ML
{

void CreateVelMANNAutoregressive(pybind11::module& module)
{
    namespace py = ::pybind11;
    namespace ML = BipedalLocomotion::ML;
    namespace System = BipedalLocomotion::System;

    py::class_<ML::velMANNAutoregressiveInput>(module, "velMANNAutoregressiveInput")
        .def(py::init())
        .def_readwrite("desired_future_base_trajectory",
                       &ML::velMANNAutoregressiveInput::desiredFutureBaseTrajectory)
        .def_readwrite("desired_future_base_directions",
                       &ML::velMANNAutoregressiveInput::desiredFutureBaseDirections)
        .def_readwrite("desired_future_base_velocities",
                       &ML::velMANNAutoregressiveInput::desiredFutureBaseVelocities)
        .def_readwrite("desired_future_base_angular_velocities",
                       &ML::velMANNAutoregressiveInput::desiredFutureBaseAngVelocities);

    py::class_<ML::velMANNAutoregressiveOutput>(module, "velMANNAutoregressiveOutput")
        .def(py::init())
        .def_readwrite("joint_positions", &ML::velMANNAutoregressiveOutput::jointsPosition)
        .def_readwrite("joint_velocities", &ML::velMANNAutoregressiveOutput::jointsVelocity)
        .def_readwrite("base_pose", &ML::velMANNAutoregressiveOutput::basePose)
        .def_readwrite("base_velocity", &ML::velMANNAutoregressiveOutput::baseVelocity)
        .def_readwrite("left_foot_pose", &ML::velMANNAutoregressiveOutput::leftFootPose)
        .def_readwrite("right_foot_pose", &ML::velMANNAutoregressiveOutput::rightFootPose)
        .def_readwrite("left_foot_velocity", &ML::velMANNAutoregressiveOutput::leftFootVelocity)
        .def_readwrite("right_foot_velocity", &ML::velMANNAutoregressiveOutput::rightFootVelocity)
        .def_readwrite("left_foot", &ML::velMANNAutoregressiveOutput::leftFoot)
        .def_readwrite("right_foot", &ML::velMANNAutoregressiveOutput::rightFoot);

    BipedalLocomotion::bindings::System::CreateAdvanceable<ML::velMANNAutoregressiveInput, //
                                                           ML::velMANNAutoregressiveOutput> //
        (module, "velMANNAutoregressive");
    py::class_<ML::velMANNAutoregressive,
               System::Advanceable<ML::velMANNAutoregressiveInput, //
                                   ML::velMANNAutoregressiveOutput>>(module, "velMANNAutoregressive")
        .def(py::init())
        .def("reset",
             py::overload_cast<Eigen::Ref<const Eigen::VectorXd>, const manif::SE3d&>(
                 &ML::velMANNAutoregressive::reset),
             py::arg("joint_positions"),
             py::arg("base_pose"))
        .def("set_robot_model", [](ML::velMANNAutoregressive& impl, ::pybind11::object& obj) {
            iDynTree::Model* cls = py::detail::swig_wrapped_pointer_to_pybind<iDynTree::Model>(obj);

            if (cls == nullptr)
            {
                throw ::pybind11::value_error("Invalid input for the function. Please provide "
                                              "an iDynTree::Model object.");
            }
            return impl.setRobotModel(*cls);
        });
}

} // namespace ML
} // namespace bindings
} // namespace BipedalLocomotion
