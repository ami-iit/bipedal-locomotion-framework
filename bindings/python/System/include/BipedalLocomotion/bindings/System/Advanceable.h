/**
 * @file Advanceable.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_ADVANCEABLE_H
#define BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_ADVANCEABLE_H

#include <string>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/System/Sink.h>
#include <BipedalLocomotion/System/Source.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{

template <class Input, class Output>
void CreateAdvanceable(pybind11::module& module, const std::string& pythonClassName)
{
    namespace py = ::pybind11;
    const std::string advanceableName = "_" + pythonClassName + "Advanceable";
    py::class_<::BipedalLocomotion::System::Advanceable<Input, Output>> //
        (module, advanceableName.c_str())
            .def(
                "initialize",
                [](::BipedalLocomotion::System::Advanceable<Input, Output>& impl,
                   std::shared_ptr<const ::BipedalLocomotion::ParametersHandler::IParametersHandler>
                       handler) -> bool { return impl.initialize(handler); },
                py::arg("handler"))
            .def("get_output", &::BipedalLocomotion::System::Advanceable<Input, Output>::getOutput)
            .def("is_output_valid",
                 &::BipedalLocomotion::System::Advanceable<Input, Output>::isOutputValid)
            .def("set_input",
                 &::BipedalLocomotion::System::Advanceable<Input, Output>::setInput,
                 py::arg("input"))
            .def("advance", &::BipedalLocomotion::System::Advanceable<Input, Output>::advance)
            .def("close", &::BipedalLocomotion::System::Advanceable<Input, Output>::close);
}

template <class Input> void CreateSink(pybind11::module& module, const std::string& pythonClassName)
{
    using Output = ::BipedalLocomotion::System::EmptySignal;
    CreateAdvanceable<Input, Output>(module, pythonClassName);

    namespace py = ::pybind11;
    const std::string sinkName = "_" + pythonClassName + "Sink";
    py::class_<::BipedalLocomotion::System::Sink<Input>,
               ::BipedalLocomotion::System::Advanceable<Input, Output>> //
        (module, sinkName.c_str());
}

template <class Output>
void CreateSource(pybind11::module& module, const std::string& pythonClassName)
{
    using Input = ::BipedalLocomotion::System::EmptySignal;
    CreateAdvanceable<Input, Output>(module, pythonClassName);

    namespace py = ::pybind11;
    const std::string sourceName = "_" + pythonClassName + "Source";
    py::class_<::BipedalLocomotion::System::Source<Output>,
               ::BipedalLocomotion::System::Advanceable<Input, Output>> //
        (module, sourceName.c_str());
}

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_ADVANCEABLE_H
