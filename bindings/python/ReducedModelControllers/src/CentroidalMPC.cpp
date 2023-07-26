/**
 * @file CentroidalMPC.cpp
 * @authors Carlotta Sartore
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_REDUCED_MODEL_CONTROLLERS_CENTROIDAL_MPC_H
#define BIPEDAL_LOCOMOTION_BINDINGS_REDUCED_MODEL_CONTROLLERS_CENTROIDAL_MPC_H

#include <BipedalLocomotion/ReducedModelControllers/CentroidalMPC.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ReducedModelControllers
{

void CreateCentroidalMPC(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::ReducedModelControllers;
    using namespace BipedalLocomotion::System;

    py::class_<CentroidalMPCOutput>(module, "CentroidalMPCState")
        .def(py::init())
        .def_readwrite("contacts", &CentroidalMPCOutput::contacts)
        .def_readwrite("next_planned_contact", &CentroidalMPCOutput::nextPlannedContact)
        .def_readwrite("com_trajectory", &CentroidalMPCOutput::comTrajectory);

    BipedalLocomotion::bindings::System::CreateSource<CentroidalMPCOutput>(module,
                                                                           "CentroidalMPCOutput");
    py::class_<CentroidalMPC, Source<CentroidalMPCOutput>>(module, "CentroidalMPC")
        .def(py::init())
        .def("set_contact_phase_list",
             py::overload_cast<const Contacts::ContactPhaseList&>(
                 &CentroidalMPC::setContactPhaseList),
             py::arg("contactPhaseList"))
        .def("set_state",
             py::overload_cast<Eigen::Ref<const Eigen::Vector3d>,
                               Eigen::Ref<const Eigen::Vector3d>,
                               Eigen::Ref<const Eigen::Vector3d>>(&CentroidalMPC::setState),
             py::arg("com"),
             py::arg("dcom"),
             py::arg("angularMomentum"))
        .def("set_reference_trajectory",
             py::overload_cast<const std::vector<Eigen::Vector3d>&,
                               const std::vector<Eigen::Vector3d>&>(
                 &CentroidalMPC::setReferenceTrajectory),
             py::arg("com"),
             py::arg("angularMomentum"));
}

} // namespace ReducedModelControllers
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_REDUCED_MODEL_CONTROLLERS_CENTROIDAL_MPC_H
