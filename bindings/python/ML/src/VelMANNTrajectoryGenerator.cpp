/**
 * @file VelMANNTrajectoryGenerator.cpp
 * @authors Evelyn D'Elia
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <iDynTree/Model/Model.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ML/VelMANNTrajectoryGenerator.h>
#include <BipedalLocomotion/bindings/ML/VelMANNTrajectoryGenerator.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>
#include <BipedalLocomotion/bindings/type_caster/swig.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ML
{

void CreateVelMANNTrajectoryGenerator(pybind11::module& module)
{
    namespace py = ::pybind11;
    namespace ML = BipedalLocomotion::ML;
    namespace System = BipedalLocomotion::System;

    py::class_<ML::VelMANNTrajectoryGeneratorInput, ML::VelMANNAutoregressiveInput>(module,
                                                                              "VelMANNTrajectoryGenera"
                                                                              "torInput")
        .def(py::init())
        .def_readwrite("merge_point_index", &ML::VelMANNTrajectoryGeneratorInput::mergePointIndex);

    py::class_<ML::VelMANNTrajectoryGeneratorOutput>(module, "VelMANNTrajectoryGeneratorOutput")
        .def(py::init())
        .def_readwrite("timestamps", &ML::VelMANNTrajectoryGeneratorOutput::timestamps)
        .def_readwrite("joint_positions", &ML::VelMANNTrajectoryGeneratorOutput::jointPositions)
        .def_readwrite("base_poses", &ML::VelMANNTrajectoryGeneratorOutput::basePoses)
        .def_readwrite("phase_list", &ML::VelMANNTrajectoryGeneratorOutput::phaseList)
        .def_readwrite("com_trajectory", &ML::VelMANNTrajectoryGeneratorOutput::comTrajectory)
        .def_readwrite("angular_momentum_trajectory",
                       &ML::VelMANNTrajectoryGeneratorOutput::angularMomentumTrajectory);

    BipedalLocomotion::bindings::System::CreateAdvanceable<ML::VelMANNTrajectoryGeneratorInput, //
                                                           ML::VelMANNTrajectoryGeneratorOutput> //
        (module, "VelMANNTrajectoryGenerator");
    py::class_<ML::VelMANNTrajectoryGenerator,
               System::Advanceable<ML::VelMANNTrajectoryGeneratorInput, //
                                   ML::VelMANNTrajectoryGeneratorOutput>>(module,
                                                                       "VelMANNTrajectoryGenerator")
        .def(py::init())
        .def("set_initial_state",
             &ML::VelMANNTrajectoryGenerator::setInitialState,
             py::arg("joint_positions"),
             py::arg("base_pose"))
        .def("set_robot_model", [](ML::VelMANNTrajectoryGenerator& impl, ::pybind11::object& obj) {
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
