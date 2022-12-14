/**
 * @file Common.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_IK_COMMON_H
#define BIPEDAL_LOCOMOTION_BINDINGS_IK_COMMON_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{
template <class T> bool setKinDyn(T& task, ::pybind11::object& obj)
{
    std::shared_ptr<iDynTree::KinDynComputations>* cls
        = pybind11::detail::swig_wrapped_pointer_to_pybind<
            std::shared_ptr<iDynTree::KinDynComputations>>(obj);

    if (cls == nullptr)
    {
        throw ::pybind11::value_error("Unable to interpret iDynTree::KinDynComputations");
    }
    return task.setKinDyn(*cls);
};

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_IK_COMMON_H
