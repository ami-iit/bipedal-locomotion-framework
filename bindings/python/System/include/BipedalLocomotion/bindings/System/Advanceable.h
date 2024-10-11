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
#include <BipedalLocomotion/System/InputPort.h>
#include <BipedalLocomotion/System/OutputPort.h>
#include <BipedalLocomotion/System/Sink.h>
#include <BipedalLocomotion/System/Source.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{

void CreateCommonDataStructure(pybind11::module& module);

template <class Input>
void CreateInputPort(pybind11::module& module, const std::string& pythonClassName)
{
    namespace py = ::pybind11;
    const std::string inputPortName = "_" + pythonClassName + "InputPort";
    py::class_<::BipedalLocomotion::System::InputPort<Input>> //
        (module, inputPortName.c_str())
            .def("set_input",
                 &::BipedalLocomotion::System::InputPort<Input>::setInput,
                 py::arg("input"));
}

template <class Output>
void CreateOutputPort(pybind11::module& module, const std::string& pythonClassName)
{
    namespace py = ::pybind11;
    const std::string outputPortName = "_" + pythonClassName + "OutputPort";
    py::class_<::BipedalLocomotion::System::OutputPort<Output>> //
        (module, outputPortName.c_str())
            .def("get_output", &::BipedalLocomotion::System::OutputPort<Output>::getOutput)
            .def("is_output_valid",
                 &::BipedalLocomotion::System::OutputPort<Output>::isOutputValid);
}

template <class Input, class Output>
void CreateAdvanceableImpl(pybind11::module& module, const std::string& pythonClassName)
{
    namespace py = ::pybind11;

    const std::string advanceableName = "_" + pythonClassName + "Advanceable";
    py::class_<::BipedalLocomotion::System::Advanceable<Input, Output>,
               ::BipedalLocomotion::System::InputPort<Input>,
               ::BipedalLocomotion::System::OutputPort<Output>> //
        (module, advanceableName.c_str(), py::multiple_inheritance())
            .def(
                "initialize",
                [](::BipedalLocomotion::System::Advanceable<Input, Output>& impl,
                   std::shared_ptr<const ::BipedalLocomotion::ParametersHandler::IParametersHandler>
                       handler) -> bool { return impl.initialize(handler); },
                py::arg("handler"))
            .def("advance", &::BipedalLocomotion::System::Advanceable<Input, Output>::advance)
            .def("close", &::BipedalLocomotion::System::Advanceable<Input, Output>::close);
}

template <class Input, class Output>
void CreateAdvanceable(pybind11::module& module, const std::string& pythonClassName)
{
    namespace py = ::pybind11;

    // the empty signal is already registered by the system module
    // please check Advanceable.cpp
    if constexpr (!std::is_same<Input, ::BipedalLocomotion::System::EmptySignal>()
                  && !std::is_same<Input, ::Eigen::VectorXd>())
    {
        ::BipedalLocomotion::bindings::System::CreateInputPort<Input>(module, pythonClassName);
    }
    if constexpr (!std::is_same<Output, ::BipedalLocomotion::System::EmptySignal>()
                  && !std::is_same<Output, ::Eigen::VectorXd>())
    {
        ::BipedalLocomotion::bindings::System::CreateOutputPort<Output>(module, pythonClassName);
    }

    if constexpr (!std::is_same<Input, ::Eigen::VectorXd>()
                  && !std::is_same<Output, ::Eigen::VectorXd>())
    {
        ::BipedalLocomotion::bindings::System::CreateAdvanceableImpl<Input, //
                                                                     Output>(module,
                                                                             pythonClassName);
    }
}

template <class Input>
void CreateSinkImpl(pybind11::module& module, const std::string& pythonClassName)
{
    using Output = ::BipedalLocomotion::System::EmptySignal;
    namespace py = ::pybind11;
    const std::string sinkName = "_" + pythonClassName + "Sink";
    py::class_<::BipedalLocomotion::System::Sink<Input>,
               ::BipedalLocomotion::System::Advanceable<Input, Output>> //
        (module, sinkName.c_str());
}

template <class Input> void CreateSink(pybind11::module& module, const std::string& pythonClassName)
{
    using Output = ::BipedalLocomotion::System::EmptySignal;
    CreateAdvanceable<Input, Output>(module, pythonClassName);

    if (!std::is_same<Input, ::Eigen::VectorXd>())
    {
        CreateSinkImpl<Input>(module, pythonClassName);
    }
}

template <class Output>
void CreateSourceImpl(pybind11::module& module, const std::string& pythonClassName)
{
    using Input = ::BipedalLocomotion::System::EmptySignal;
    namespace py = ::pybind11;
    const std::string sourceName = "_" + pythonClassName + "Source";
    py::class_<::BipedalLocomotion::System::Source<Output>,
               ::BipedalLocomotion::System::Advanceable<Input, Output>> //
        (module, sourceName.c_str());
}

template <class Output>
void CreateSource(pybind11::module& module, const std::string& pythonClassName)
{
    using Input = ::BipedalLocomotion::System::EmptySignal;
    CreateAdvanceable<Input, Output>(module, pythonClassName);

    if (!std::is_same<Output, ::Eigen::VectorXd>())
    {
        CreateSourceImpl<Output>(module, pythonClassName);
    }
}

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_ADVANCEABLE_H
