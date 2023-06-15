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
#include <BipedalLocomotion/bindings/type_caster/swig.h>

#include <iDynTree/KinDynComputations.h>

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

template <class LinearTask> bool setKinDyn(LinearTask& task, ::pybind11::object& obj)
{
    static_assert(std::is_base_of<BipedalLocomotion::System::LinearTask, LinearTask>::value,
                  "This function can be called only to set a KinDynComputations object to a linear "
                  "task.");

    std::shared_ptr<iDynTree::KinDynComputations>* cls
        = pybind11::detail::swig_wrapped_pointer_to_pybind<
            std::shared_ptr<iDynTree::KinDynComputations>>(obj);

    if (cls == nullptr)
    {
        throw ::pybind11::value_error("Invalid input for the function. Please provide an "
                                      "iDynTree::KinDynComputations object.");
    }

    return task.setKinDyn(*cls);
};


void CreateLinearTask(pybind11::module& module);

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_LINEAR_TASK_H
