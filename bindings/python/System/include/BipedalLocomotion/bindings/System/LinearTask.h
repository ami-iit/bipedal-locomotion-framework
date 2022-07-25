/**
 * @file LinearTask.h
 * @authors Paolo Maria Viceconte, Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_LINEAR_TASK_H
#define BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_LINEAR_TASK_H

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/System/LinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{

template <class LinearTaskBase = BipedalLocomotion::System::LinearTask>
class LinearTaskTrampoline : public LinearTaskBase
{
public:
    using LinearTaskBase::LinearTaskBase;

    bool
    setVariablesHandler(const BipedalLocomotion::System::VariablesHandler& variablesHandler) override
    {
        PYBIND11_OVERLOAD_NAME(bool,
                               LinearTaskBase,
                               "set_variables_handler",
                               setVariablesHandler,
                               variablesHandler);
    }

    bool
    initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override
    {
        PYBIND11_OVERLOAD(bool, LinearTaskBase, initialize, paramHandler);
    }

    bool update() override
    {
        PYBIND11_OVERLOAD(bool, LinearTaskBase, update);
    }

    std::size_t size() const override
    {
        PYBIND11_OVERLOAD_PURE(std::size_t, LinearTaskBase, size);
    }

    typename LinearTaskBase::Type type() const override
    {
        PYBIND11_OVERLOAD_PURE(typename LinearTaskBase::Type, LinearTaskBase, type);
    }

    bool isValid() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, LinearTaskBase, "is_valid", isValid);
    }
};

void CreateLinearTask(pybind11::module& module);

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_LINEAR_TASK_H
