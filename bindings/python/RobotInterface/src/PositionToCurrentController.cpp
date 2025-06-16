/**
 * @file PositionToCurrentController.cpp
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/RobotInterface/PositionToCurrentController.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <BipedalLocomotion/bindings/RobotInterface/PositionToCurrentController.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{

void CreatePositionToCurrentController(pybind11::module& module)
{
    using namespace ::BipedalLocomotion::RobotInterface;
    using namespace ::BipedalLocomotion::System;
    namespace py = ::pybind11;

    py::class_<PositionToCurrentControllerInput>(module, "PositionToCurrentControllerInput")
        .def(py::init())
        .def_readwrite("reference_position", &PositionToCurrentControllerInput::referencePosition)
        .def_readwrite("feedback_position", &PositionToCurrentControllerInput::feedbackPosition)
        .def_readwrite("feedback_velocity", &PositionToCurrentControllerInput::feedbackVelocity);

    BipedalLocomotion::bindings::System::CreateAdvanceable<PositionToCurrentControllerInput,
                                                           ::Eigen::VectorXd>( //
        module,
        "PositionToCurrentController");

    py::class_<::BipedalLocomotion::RobotInterface::PositionToCurrentController,
               ::BipedalLocomotion::System::Advanceable<PositionToCurrentControllerInput,
                                                        ::Eigen::VectorXd>> //
        (module, "PositionToCurrentController").def(py::init());
}
} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
