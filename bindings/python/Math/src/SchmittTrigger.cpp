/**
 * @file SchmittTrigger.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

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

    py::class_<SchmittTriggerState>(module, "SchmittTriggerState")
        .def(py::init<>())
        .def_readwrite("state", &SchmittTriggerState::state)
        .def_readwrite("switch_time", &SchmittTriggerState::switchTime)
        .def_readwrite("edge_time", &SchmittTriggerState::edgeTime);

    py::class_<SchmittTriggerInput>(module, "SchmittTriggerInput")
        .def(py::init<>())
        .def_readwrite("time", &SchmittTriggerInput::time)
        .def_readwrite("raw_value", &SchmittTriggerInput::rawValue);

    BipedalLocomotion::bindings::System::CreateAdvanceable<
        BipedalLocomotion::Math::SchmittTriggerInput,
        BipedalLocomotion::Math::SchmittTriggerState>(module, "SchmittTrigger");

    py::class_<SchmittTrigger,
               BipedalLocomotion::System::Advanceable<SchmittTriggerInput, //
                                                      SchmittTriggerState>>
        schmittTrigger(module, "SchmittTrigger");

    py::class_<SchmittTrigger::Params>(schmittTrigger, "Params")
        .def(py::init())
        .def_readwrite("on_threshold", &SchmittTrigger::Params::onThreshold)
        .def_readwrite("off_threshold", &SchmittTrigger::Params::offThreshold)
        .def_readwrite("switch_on_after", &SchmittTrigger::Params::switchOnAfter)
        .def_readwrite("switch_off_after", &SchmittTrigger::Params::switchOffAfter)
        .def_readwrite("time_comparison_threshold", &SchmittTrigger::Params::timeComparisonThreshold);

    schmittTrigger.def(py::init())
        .def("set_state", &SchmittTrigger::setState, py::arg("state"))
        .def("initialize",
             py::overload_cast<const SchmittTrigger::Params&>(&SchmittTrigger::initialize),
             py::arg("parameters"));
}
} // namespace Math
} // namespace bindings
} // namespace BipedalLocomotion
