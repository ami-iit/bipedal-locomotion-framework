/**
 * @file MANNTrajectoryGenerator.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <iDynTree/Model.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ML/MANNTrajectoryGenerator.h>
#include <BipedalLocomotion/bindings/ML/MANNTrajectoryGenerator.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>
#include <BipedalLocomotion/bindings/type_caster/swig.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ML
{

void CreateMANNTrajectoryGenerator(pybind11::module& module)
{
    namespace py = ::pybind11;
    namespace ML = BipedalLocomotion::ML;
    namespace System = BipedalLocomotion::System;

    py::class_<ML::MANNTrajectoryGeneratorInput, ML::MANNAutoregressiveInput>(module,
                                                                              "MANNTrajectoryGenera"
                                                                              "torInput")
        .def(py::init())
        .def_readwrite("merge_point_index", &ML::MANNTrajectoryGeneratorInput::mergePointIndex);

    py::class_<ML::MANNTrajectoryGeneratorOutput>(module, "MANNTrajectoryGeneratorOutput")
        .def(py::init())
        .def_readwrite("timestamps", &ML::MANNTrajectoryGeneratorOutput::timestamps)
        .def_readwrite("joint_positions", &ML::MANNTrajectoryGeneratorOutput::jointPositions)
        .def_readwrite("base_poses", &ML::MANNTrajectoryGeneratorOutput::basePoses)
        .def_readwrite("phase_list", &ML::MANNTrajectoryGeneratorOutput::phaseList)
        .def_readwrite("com_trajectory", &ML::MANNTrajectoryGeneratorOutput::comTrajectory)
        .def_readwrite("angular_momentum_trajectory",
                       &ML::MANNTrajectoryGeneratorOutput::angularMomentumTrajectory);

    BipedalLocomotion::bindings::System::CreateAdvanceable<ML::MANNTrajectoryGeneratorInput, //
                                                           ML::MANNTrajectoryGeneratorOutput> //
        (module, "MANNTrajectoryGenerator");
    py::class_<ML::MANNTrajectoryGenerator,
               System::Advanceable<ML::MANNTrajectoryGeneratorInput, //
                                   ML::MANNTrajectoryGeneratorOutput>>(module,
                                                                       "MANNTrajectoryGenerator")
        .def(py::init())
        .def("set_initial_state",
             &ML::MANNTrajectoryGenerator::setInitialState,
             py::arg("joint_positions"),
             py::arg("base_pose"))
        .def("set_robot_model", [](ML::MANNTrajectoryGenerator& impl, ::pybind11::object& obj) {
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
