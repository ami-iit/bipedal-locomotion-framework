/**
 * @file QPTSID.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TSID/QPFixedBaseTSID.h>
#include <BipedalLocomotion/TSID/QPTSID.h>

#include <BipedalLocomotion/bindings/TSID/QPTSID.h>
#include <BipedalLocomotion/bindings/type_caster/swig.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateQPTSID(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;
    using namespace BipedalLocomotion::ParametersHandler;

    py::class_<QPTSID, TaskSpaceInverseDynamics>(module, "QPTSID")
        .def(py::init())
        .def_static(
            "build",
            [](std::shared_ptr<const IParametersHandler> handler,
               py::object& obj) -> TaskSpaceInverseDynamicsProblem {
                // get the kindyn computation object from the swig binsings
                std::shared_ptr<iDynTree::KinDynComputations>* cls
                    = py::detail::swig_wrapped_pointer_to_pybind<
                        std::shared_ptr<iDynTree::KinDynComputations>>(obj);

                if (cls == nullptr)
                {
                    throw ::pybind11::value_error("Invalid input for the function. Please provide "
                                                  "an iDynTree::KinDynComputations object.");
                }

                return QPTSID::build(handler, *cls);
            },
            py::arg("param_handler"),
            py::arg("kin_dyn"));
}

void CreateQPFixedBaseTSID(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;

    py::class_<QPFixedBaseTSID, QPTSID>(module, "QPFixedBaseTSID")
        .def(py::init())
        .def(
            "set_kin_dyn",
            [](QPFixedBaseTSID& impl, ::pybind11::object& obj) -> bool {
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

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
