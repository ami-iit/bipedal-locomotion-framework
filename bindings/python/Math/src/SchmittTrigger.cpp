/**
 * @file SchmittTrigger.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>

#include <BipedalLocomotion/Math/SchmittTrigger.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/bindings/Math/SchmittTrigger.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Math
{

void CreateSchmittTrigger(pybind11::module& module)
{
    using namespace BipedalLocomotion::Math;
    namespace py = ::pybind11;

    py::class_<SchmittTriggerOutput>(module, "SchmittTriggerOutput")
        .def(py::init([](bool state,
                         std::chrono::nanoseconds switchTime,
                         std::chrono::nanoseconds edgeTime) -> SchmittTriggerOutput {
                 return SchmittTriggerOutput{std::move(state),
                                             std::move(switchTime),
                                             std::move(edgeTime)};
             }),
             py::arg("state") = false,
             py::arg("switch_time") = std::chrono::nanoseconds::zero(),
             py::arg("edge_time") = std::chrono::nanoseconds::zero())
        .def_readwrite("state", &SchmittTriggerOutput::state)
        .def_readwrite("switch_time", &SchmittTriggerOutput::switchTime)
        .def_readwrite("edge_time", &SchmittTriggerOutput::edgeTime);

    py::class_<SchmittTriggerState, SchmittTriggerOutput>(module, "SchmittTriggerState")
        .def(py::init([](bool state,
                         std::chrono::nanoseconds switchTime,
                         std::chrono::nanoseconds edgeTime,
                         std::chrono::nanoseconds timer,
                         std::chrono::nanoseconds risingEdgeTimeInstant,
                         std::chrono::nanoseconds fallingEdgeTimeInstant,
                         bool risingDetected,
                         bool fallingDetected) -> SchmittTriggerState {
                 return SchmittTriggerState{std::move(state),
                                            std::move(switchTime),
                                            std::move(edgeTime),
                                            std::move(timer),
                                            std::move(risingEdgeTimeInstant),
                                            std::move(fallingEdgeTimeInstant),
                                            std::move(risingDetected),
                                            std::move(fallingDetected)};
             }),
             py::arg("state") = false,
             py::arg("switch_time") = std::chrono::nanoseconds::zero(),
             py::arg("edge_time") = std::chrono::nanoseconds::zero(),
             py::arg("timer") = std::chrono::nanoseconds::zero(),
             py::arg("rising_edge_time_instant") = std::chrono::nanoseconds::zero(),
             py::arg("falling_edge_time_instant") = std::chrono::nanoseconds::zero(),
             py::arg("rising_detected") = false,
             py::arg("falling_detected") = false)
        .def_readwrite("timer", &SchmittTriggerState::timer)
        .def_readwrite("rising_edge_time_instant", &SchmittTriggerState::risingEdgeTimeInstant)
        .def_readwrite("falling_edge_time_instant", &SchmittTriggerState::fallingEdgeTimeInstant)
        .def_readwrite("rising_detected", &SchmittTriggerState::risingDetected)
        .def_readwrite("falling_detected", &SchmittTriggerState::fallingDetected);

    py::class_<SchmittTriggerInput>(module, "SchmittTriggerInput")
        .def(py::init([](std::chrono::nanoseconds time, double rawValue) -> SchmittTriggerInput {
                 return SchmittTriggerInput{std::move(time), std::move(rawValue)};
             }),
             py::arg("time") = std::chrono::nanoseconds::zero(),
             py::arg("raw_value") = 0.0)
        .def_readwrite("time", &SchmittTriggerInput::time)
        .def_readwrite("raw_value", &SchmittTriggerInput::rawValue);

    BipedalLocomotion::bindings::System::CreateAdvanceable<
        BipedalLocomotion::Math::SchmittTriggerInput,
        BipedalLocomotion::Math::SchmittTriggerOutput>(module, "SchmittTrigger");

    py::class_<SchmittTrigger,
               ::BipedalLocomotion::System::Advanceable<SchmittTriggerInput, //
                                                        SchmittTriggerOutput>>
        schmittTrigger(module, "SchmittTrigger");

    py::class_<SchmittTrigger::Params>(schmittTrigger, "Params")
        .def(py::init([](double onThreshold,
                         double offThreshold,
                         std::chrono::nanoseconds switchOnAfter,
                         std::chrono::nanoseconds switchOffAfter) -> SchmittTrigger::Params {
                 SchmittTrigger::Params params;
                 params.onThreshold = std::move(onThreshold);
                 params.offThreshold = std::move(offThreshold);
                 params.switchOnAfter = std::move(switchOnAfter);
                 params.switchOffAfter = std::move(switchOffAfter);
                 return params;
             }),
             py::arg("on_threshold") = 0.0,
             py::arg("off_threshold") = 0.0,
             py::arg("switch_on_after") = std::chrono::nanoseconds::zero(),
             py::arg("switch_off_after") = std::chrono::nanoseconds::zero())
        .def_readwrite("on_threshold", &SchmittTrigger::Params::onThreshold)
        .def_readwrite("off_threshold", &SchmittTrigger::Params::offThreshold)
        .def_readwrite("switch_on_after", &SchmittTrigger::Params::switchOnAfter)
        .def_readwrite("switch_off_after", &SchmittTrigger::Params::switchOffAfter);

    schmittTrigger.def(py::init())
        .def("set_state", &SchmittTrigger::setState, py::arg("state"))
        .def("get_state", &SchmittTrigger::getState)
        .def("initialize",
             py::overload_cast<const SchmittTrigger::Params&>(&SchmittTrigger::initialize),
             py::arg("parameters"));
}
} // namespace Math
} // namespace bindings
} // namespace BipedalLocomotion
