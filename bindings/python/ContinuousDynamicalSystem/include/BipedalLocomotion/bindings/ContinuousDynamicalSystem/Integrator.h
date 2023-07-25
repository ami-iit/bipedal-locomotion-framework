/**
 * @file Integrator.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_CONTINUOUS_DYNAMICAL_SYSTEM_INTEGRATOR_H
#define BIPEDAL_LOCOMOTION_BINDINGS_CONTINUOUS_DYNAMICAL_SYSTEM_INTEGRATOR_H

#include <chrono>
#include <memory>
#include <string>

#include <BipedalLocomotion/ContinuousDynamicalSystem/FixedStepIntegrator.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/Integrator.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/RK4.h>

#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{

template <typename _Derived, typename... _Args>
void CreateIntegrator(pybind11::module& module, const std::string& name)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::ContinuousDynamicalSystem;

    const std::string completeName = "_" + name + "BaseIntegrator";
    py::class_<Integrator<_Derived>, _Args...>(module, completeName.c_str())
        .def("set_dynamical_system",
             &Integrator<_Derived>::setDynamicalSystem,
             py::arg("dynamical_system"))
        .def("dynamical_system",
             [](const Integrator<_Derived>& impl) {
                 auto ptr = impl.dynamicalSystem().lock();
                 if (ptr == nullptr)
                 {
                     throw py::value_error("The Dynamical system is not valid.");
                 }
                 return ptr;
             })
        .def_property(
            "dynamical_system",
            [](const Integrator<_Derived>& impl) {
                auto ptr = impl.dynamicalSystem().lock();
                if (ptr == nullptr)
                {
                    throw py::value_error("The Dynamical system is not valid.");
                }
                return ptr;
            },
            [](Integrator<_Derived>& impl,
               std::shared_ptr<typename Integrator<_Derived>::DynamicalSystem> dynamicalSystem) {
                if (!impl.setDynamicalSystem(dynamicalSystem))
                {
                    throw py::value_error("The Dynamical system is not valid.");
                }
            })
        .def("get_solution",
             [](const Integrator<_Derived>& impl) ->
             typename Integrator<_Derived>::State::underlying_tuple {
                 return impl.getSolution().to_tuple();
             })
        .def("integrate",
             &Integrator<_Derived>::integrate,
             py::arg("initial_time"),
             py::arg("final_time"));
}

template <typename _Derived, typename... _Args>
void CreateFixedStepIntegrator(pybind11::module& module, const std::string& name)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::ContinuousDynamicalSystem;

    const std::string completeName = "_" + name + "FixedStepIntegrator";
    CreateIntegrator<FixedStepIntegrator<_Derived>>(module, name);
    py::class_<FixedStepIntegrator<_Derived>,
               Integrator<FixedStepIntegrator<_Derived>>,
               _Args...>(module, completeName.c_str())
        .def("set_integration_step",
             &FixedStepIntegrator<_Derived>::setIntegrationStep,
             py::arg("dT"))
        .def("get_integration_step", &FixedStepIntegrator<_Derived>::getIntegrationStep)
        .def_property("integration_step",
                      &FixedStepIntegrator<_Derived>::getIntegrationStep,
                      [](FixedStepIntegrator<_Derived>& impl, const std::chrono::nanoseconds& dt) {
                          if (!impl.setIntegrationStep(dt))
                          {
                              throw py::value_error("Invalid integration step.");
                          }
                      });
}

template <typename _DynamicalSystem, typename... _Args>
void CreateForwardEulerIntegrator(pybind11::module& module, const std::string& name)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::ContinuousDynamicalSystem;

    const std::string completeName = name + "ForwardEulerIntegrator";
    CreateFixedStepIntegrator<ForwardEuler<_DynamicalSystem>>(module, completeName);

    py::class_<ForwardEuler<_DynamicalSystem>,
               FixedStepIntegrator<ForwardEuler<_DynamicalSystem>>,
               _Args...>(module, completeName.c_str())
        .def(py::init());
}

template <typename _DynamicalSystem, typename... _Args>
void CreateRK4Integrator(pybind11::module& module, const std::string& name)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::ContinuousDynamicalSystem;

    const std::string completeName = name + "RK4Integrator";
    CreateFixedStepIntegrator<RK4<_DynamicalSystem>>(module, completeName);

    py::class_<RK4<_DynamicalSystem>,
               FixedStepIntegrator<RK4<_DynamicalSystem>>,
               _Args...>(module, completeName.c_str())
        .def(py::init());
}

} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_CONTINUOUS_DYNAMICAL_SYSTEM_INTEGRATOR_H
