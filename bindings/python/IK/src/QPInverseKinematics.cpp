/**
 * @file QPInverseKinematics.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/QPFixedBaseInverseKinematics.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Source.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <BipedalLocomotion/bindings/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/bindings/type_caster/swig.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateQPInverseKinematics(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;
    using namespace BipedalLocomotion::ParametersHandler;
    using namespace BipedalLocomotion::System;

    py::class_<QPInverseKinematics, IntegrationBasedIK>(module, "QPInverseKinematics")
        .def(py::init())
        .def_static(
            "build",
            [](std::shared_ptr<const IParametersHandler> handler, py::object& obj)
                -> std::pair<VariablesHandler, std::unique_ptr<QPInverseKinematics>> {

                // get the kindyn computation object from the swig binsings
                std::shared_ptr<iDynTree::KinDynComputations>* cls
                    = py::detail::swig_wrapped_pointer_to_pybind<
                        std::shared_ptr<iDynTree::KinDynComputations>>(obj);

                if (cls == nullptr)
                {
                    throw ::pybind11::value_error("Invalid input for the function. Please provide "
                                                  "an iDynTree::KinDynComputations object.");
                }

                return QPInverseKinematics::build(handler, *cls);
            },
            py::arg("param_handler"),
            py::arg("kin_dyn"));

    py::class_<QPFixedBaseInverseKinematics, QPInverseKinematics>(module,
                                                                  "QPFixedBaseInverseKinematics")
        .def(py::init())
        .def(
            "set_kin_dyn",
            [](QPFixedBaseInverseKinematics& impl, ::pybind11::object& obj) -> bool {
                std::shared_ptr<iDynTree::KinDynComputations>* cls
                    = pybind11::detail::swig_wrapped_pointer_to_pybind<
                        std::shared_ptr<iDynTree::KinDynComputations>>(obj);

                if (cls == nullptr)
                {
                    throw ::pybind11::value_error("Invalid input for the function. Please provide "
                                                  "an iDynTree::KinDynComputations object.");
                }

                return impl.setKinDyn(*cls);
            },
            py::arg("kin_dyn"));
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
