/**
 * @file PositionToCurrentController.cpp
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/JointLevelControllers/PositionToCurrentController.h>
#include <BipedalLocomotion/JointLevelControllers/PositionToJointController.h>
#include <BipedalLocomotion/JointLevelControllers/PositionToTorqueController.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <BipedalLocomotion/bindings/JointLevelControllers/PositionToJointController.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace JointLevelControllers
{

void CreatePositionToJointController(pybind11::module& module)
{
    using namespace ::BipedalLocomotion::JointLevelControllers;
    using namespace ::BipedalLocomotion::System;
    namespace py = ::pybind11;

    py::class_<PositionToJointControllerInput>(module, "PositionToJointControllerInput")
        .def(py::init())
        .def_readwrite("reference_position", &PositionToJointControllerInput::referencePosition)
        .def_readwrite("feedback_position", &PositionToJointControllerInput::feedbackPosition)
        .def_readwrite("feedback_velocity", &PositionToJointControllerInput::feedbackVelocity);

    BipedalLocomotion::bindings::System::CreateAdvanceable<PositionToJointControllerInput,
                                                           ::Eigen::VectorXd>( //
        module,
        "PositionToJointController");
    py::class_<::BipedalLocomotion::JointLevelControllers::PositionToJointController,
               ::BipedalLocomotion::System::Advanceable<PositionToJointControllerInput,
                                                        ::Eigen::VectorXd>> //
        (module, "PositionToJointController");
}

void CreatePositionToCurrentController(pybind11::module& module)
{

    using namespace ::BipedalLocomotion::JointLevelControllers;
    namespace py = ::pybind11;
    py::class_<::BipedalLocomotion::JointLevelControllers::PositionToCurrentController,
               ::BipedalLocomotion::JointLevelControllers::PositionToJointController> //
        (module, "PositionToCurrentController").def(py::init());
}

void CreatePositionToTorqueController(pybind11::module& module)
{

    using namespace ::BipedalLocomotion::JointLevelControllers;
    namespace py = ::pybind11;
    py::class_<::BipedalLocomotion::JointLevelControllers::PositionToTorqueController,
               ::BipedalLocomotion::JointLevelControllers::PositionToJointController> //
        (module, "PositionToTorqueController").def(py::init());
}
} // namespace JointLevelControllers
} // namespace bindings
} // namespace BipedalLocomotion
