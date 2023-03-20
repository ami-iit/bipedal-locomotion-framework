/**
 * @file ContactDetectors.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021-2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ContactDetectors/ContactDetector.h>
#include <BipedalLocomotion/ContactDetectors/FixedFootDetector.h>
#include <BipedalLocomotion/ContactDetectors/SchmittTriggerDetector.h>
#include <BipedalLocomotion/Math/SchmittTrigger.h>

#include <BipedalLocomotion/bindings/Contacts/ContactDetectors.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Contacts
{

void CreateContactDetector(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Contacts;
    using namespace BipedalLocomotion::System;
    using namespace BipedalLocomotion::ParametersHandler;

    BipedalLocomotion::bindings::System::CreateSource<EstimatedContactList>(module,
                                                                            "ContactDetector");
    py::class_<ContactDetector, Source<EstimatedContactList>>(module, "ContactDetector");
}

void CreateSchmittTriggerDetector(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Contacts;
    using namespace BipedalLocomotion::Math;

    py::class_<SchmittTriggerDetector, ContactDetector>(module, "SchmittTriggerDetector")
        .def(py::init())
        .def("reset_contacts", &SchmittTriggerDetector::resetContacts)
        .def("get",
             py::overload_cast<const std::string&>(&SchmittTriggerDetector::get, py::const_),
             py::arg("contact_name"))
        .def("set_timed_trigger_input",
             py::overload_cast<const std::string&, const SchmittTriggerInput&>(
                 &SchmittTriggerDetector::setTimedTriggerInput),
             py::arg("contact_name"),
             py::arg("input"))
        .def("set_timed_trigger_inputs",
             py::overload_cast<const std::unordered_map<std::string, //
                                                        Math::SchmittTriggerInput>&>(
                 &SchmittTriggerDetector::setTimedTriggerInputs),
             py::arg("timed_inputs"))
        .def("add_contact",
             &SchmittTriggerDetector::addContact,
             py::arg("contact_name"),
             py::arg("initial_state"),
             py::arg("params"))
        .def("reset_state",
             &SchmittTriggerDetector::resetState,
             py::arg("contact_name"),
             py::arg("state"))
        .def("reset_contact",
             &SchmittTriggerDetector::resetContact,
             py::arg("contact_name"),
             py::arg("state"),
             py::arg("params"))
        .def("remove_contact", &SchmittTriggerDetector::removeContact, py::arg("contact_name"));
}

void CreateFixedFootDetector(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Contacts;

    py::class_<FixedFootDetector, ContactDetector>(module, "FixedFootDetector")
        .def(py::init())
        .def("get_fixed_foot", &FixedFootDetector::getFixedFoot)
        .def("set_contact_phase_list",
             &FixedFootDetector::setContactPhaseList,
             py::arg("phase_list"))
        .def("reset_time", &FixedFootDetector::resetTime, py::arg("time"));
}

} // namespace Contacts
} // namespace bindings
} // namespace BipedalLocomotion
