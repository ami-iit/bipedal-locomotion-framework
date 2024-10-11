/**
 * @file MannAutoregressive.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <iDynTree/Model.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ML/MANNAutoregressive.h>
#include <BipedalLocomotion/bindings/ML/MANNAutoregressive.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>
#include <BipedalLocomotion/bindings/type_caster/swig.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ML
{

void CreateMANNAutoregressive(pybind11::module& module)
{
    namespace py = ::pybind11;
    namespace ML = BipedalLocomotion::ML;
    namespace System = BipedalLocomotion::System;

    py::class_<ML::MANNAutoregressiveInput>(module, "MANNAutoregressiveInput")
        .def(py::init())
        .def_readwrite("desired_future_base_trajectory",
                       &ML::MANNAutoregressiveInput::desiredFutureBaseTrajectory)
        .def_readwrite("desired_future_base_velocities",
                       &ML::MANNAutoregressiveInput::desiredFutureBaseVelocities)
        .def_readwrite("desired_future_facing_directions",
                       &ML::MANNAutoregressiveInput::desiredFutureFacingDirections);

    py::class_<ML::MANNAutoregressiveOutput>(module, "MANNAutoregressiveOutput")
        .def(py::init())
        .def_readwrite("joint_positions", &ML::MANNAutoregressiveOutput::jointsPosition)
        .def_readwrite("base_pose", &ML::MANNAutoregressiveOutput::basePose)
        .def_readwrite("left_foot", &ML::MANNAutoregressiveOutput::leftFoot)
        .def_readwrite("right_foot", &ML::MANNAutoregressiveOutput::rightFoot);

    BipedalLocomotion::bindings::System::CreateAdvanceable<ML::MANNAutoregressiveInput, //
                                                           ML::MANNAutoregressiveOutput> //
        (module, "MANNAutoregressive");
    py::class_<ML::MANNAutoregressive,
               System::Advanceable<ML::MANNAutoregressiveInput, //
                                   ML::MANNAutoregressiveOutput>>(module, "MANNAutoregressive")
        .def(py::init())
        .def("reset",
             py::overload_cast<Eigen::Ref<const Eigen::VectorXd>, const manif::SE3d&>(
                 &ML::MANNAutoregressive::reset),
             py::arg("joint_positions"),
             py::arg("base_pose"))
        .def("set_robot_model", [](ML::MANNAutoregressive& impl, ::pybind11::object& obj) {
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
