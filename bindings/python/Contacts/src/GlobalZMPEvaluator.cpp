/**
 * @file GlobalZMPEvaluator.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/Contacts/GlobalZMPEvaluator.h>

#include <BipedalLocomotion/bindings/Contacts/GlobalZMPEvaluator.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Contacts
{

void CreateGlobalZMPEvaluator(pybind11::module& module)
{
    namespace py = ::pybind11;
    namespace Contacts = ::BipedalLocomotion::Contacts;
    namespace System = ::BipedalLocomotion::System;

    BipedalLocomotion::bindings::System::CreateAdvanceable<std::vector<Contacts::ContactWrench>, //
                                                           Eigen::Vector3d>(module,
                                                                            "GlobalZMPEvaluator");
    py::class_<Contacts::GlobalZMPEvaluator,
               System::Advanceable<std::vector<Contacts::ContactWrench>, //
                                   Eigen::Vector3d>>(module, "GlobalZMPEvaluator")
        .def(py::init());
}

} // namespace Contacts
} // namespace bindings
} // namespace BipedalLocomotion
