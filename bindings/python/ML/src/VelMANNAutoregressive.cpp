/**
 * @file VelMANNAutoregressive.cpp
 * @authors Evelyn D'Elia
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <iDynTree/Model/Model.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ML/VelMANNAutoregressive.h>
#include <BipedalLocomotion/bindings/ML/VelMANNAutoregressive.h>
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

    py::class_<ML::VelMANNAutoregressiveInput>(module, "VelMANNAutoregressiveInput")
        .def(py::init())
        .def_readwrite("desired_future_base_trajectory",
                       &ML::VelMANNAutoregressiveInput::desiredFutureBaseTrajectory)
        .def_readwrite("desired_future_base_directions",
                       &ML::VelMANNAutoregressiveInput::desiredFutureBaseDirections)
        .def_readwrite("desired_future_base_velocities",
                       &ML::VelMANNAutoregressiveInput::desiredFutureBaseVelocities)
        .def_readwrite("desired_future_base_angular_velocities",
                       &ML::VelMANNAutoregressiveInput::desiredFutureBaseAngVelocities);

    py::class_<ML::VelMANNAutoregressiveOutput>(module, "VelMANNAutoregressiveOutput")
        .def(py::init())
        .def_readwrite("joint_positions", &ML::VelMANNAutoregressiveOutput::jointsPosition)
        .def_readwrite("joint_velocities", &ML::VelMANNAutoregressiveOutput::jointsVelocity)
        .def_readwrite("base_pose", &ML::VelMANNAutoregressiveOutput::basePose)
        .def_readwrite("base_velocity", &ML::VelMANNAutoregressiveOutput::baseVelocity)
        .def_readwrite("left_foot_pose", &ML::VelMANNAutoregressiveOutput::leftFootPose)
        .def_readwrite("right_foot_pose", &ML::VelMANNAutoregressiveOutput::rightFootPose)
        .def_readwrite("left_foot_velocity", &ML::VelMANNAutoregressiveOutput::leftFootVelocity)
        .def_readwrite("right_foot_velocity", &ML::VelMANNAutoregressiveOutput::rightFootVelocity)
        .def_readwrite("left_foot", &ML::VelMANNAutoregressiveOutput::leftFoot)
        .def_readwrite("right_foot", &ML::VelMANNAutoregressiveOutput::rightFoot);

    BipedalLocomotion::bindings::System::CreateAdvanceable<ML::VelMANNAutoregressiveInput, //
                                                           ML::VelMANNAutoregressiveOutput> //
        (module, "VelMANNAutoregressive");
    py::class_<ML::VelMANNAutoregressive,
               System::Advanceable<ML::VelMANNAutoregressiveInput, //
                                   ML::VelMANNAutoregressiveOutput>>(module, "VelMANNAutoregressive")
        .def(py::init())
        .def("reset",
             py::overload_cast<Eigen::Ref<const Eigen::VectorXd>, const manif::SE3d&>(
                 &ML::VelMANNAutoregressive::reset),
             py::arg("joint_positions"),
             py::arg("base_pose"))
        .def("set_robot_model", [](ML::VelMANNAutoregressive& impl, ::pybind11::object& obj) {
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
