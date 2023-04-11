/**
 * @file CentroidalMPC.cpp
 * @authors Carlotta Sartore
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_REDUCED_MODEL_CONTROLLERS_CENTROIDAL_MPC_H
#define BIPEDAL_LOCOMOTION_BINDINGS_REDUCED_MODEL_CONTROLLERS_CENTROIDAL_MPC_H

#include <pybind11/pybind11.h>
#include <pybind11/chrono>
#include <pybind11/stl.h>
#include <unordered_map>

#include <casadi/casadi.hpp>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Math/LinearizedFrictionCone.h>
#include <BipedalLocomotion/ReducedModelControllers/CentroidalMPC.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ReducedModelControllers
{

void CreateCentroidalMPC(pybind11::module& module)
{   
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Contacts;
    using namespace BipedalLocomotion::System;
    using namespace BipedalLocomotion::ParametersHandler;
    BipedalLocomotion::bindings::System::CreateSource<CentroidalMPCState>(module,
                                                                        "CentroidalMPCState");
    py::class_ <CentroidalMPC, CentroidalMPCState>(module, "CentroidalMPC")
        .def(py.init())
        .def("initialize", py::overload_cast<std::weak_ptr<const ParametersHandler::IParametersHandler>(&CentroidalMPC::initialize), 
            py::arg("handler"))
        .def("set_contact_phase_list", py::overload_cast<const Contacts::ContactPhaseList&>(&CentroidalMPC::setContactPhaseList),
            py::arg("contactPhaseList") )
        .def("set_state", py::overload_cast<Eigen::Ref<const Eigen::Vector3d>,Eigen::Ref<const Eigen::Vector3d>,Eigen::Ref<const Eigen::Vector3d>,std::optional<Eigen::Ref<const Eigen::Vector3d>>>(&CentroidalMPC::setState), 
            py::arg("com"), 
            py::arg("dcom"),
            py::arg("angularMomentum"), 
            py::arg("externalWrench"))
        .def("set_reference_trajectory", py::overload_cast<Eigen::Ref<const Eigen::MatrixXd>>(&CentroidalMPC::setReferenceTrajectory), 
            py::arg("com"))
        .def("get_output", &CentroidalMPC::getOutput)
        .def("is_output_valid", &CentroidalMPC::isOutputValid)
        .def("advance", &CentroidalMPC::advance)
}

} // namespace ReducedModelControllers
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_REDUCED_MODEL_CONTROLLERS_CENTROIDAL_MPC_H
