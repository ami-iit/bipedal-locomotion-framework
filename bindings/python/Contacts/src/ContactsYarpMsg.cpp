/**
 * @file ContactsYarpMsg.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Contacts/ContactListMapYarpMsg.h>
#include <BipedalLocomotion/Contacts/ContactListYarpMsg.h>
#include <BipedalLocomotion/Contacts/PlannedContactYarpMsg.h>

#include <BipedalLocomotion/bindings/Contacts/ContactsYarpMsg.h>
#include <BipedalLocomotion/bindings/YarpUtilities/BufferedPort.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Contacts
{
void CreatePlannedContactYarpMsg(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace ::BipedalLocomotion::Contacts;

    py::class_<PlannedContactYarpMsg, PlannedContact>(module, "PlannedContactYarpMsg")
        .def(py::init<const PlannedContact&>())
        .def("from_planned_contact",
             &PlannedContactYarpMsg::fromPlannedContact,
             py::arg("planned_contact"))
        .def("__repr__", &PlannedContactYarpMsg::toString)
        .def("to_string", &PlannedContactYarpMsg::toString);

    using ::BipedalLocomotion::bindings::YarpUtilities::CreateBufferedPort;
    CreateBufferedPort<PlannedContactYarpMsg>(module, "BufferedPlannedContact");
}

void CreateContactListYarpMsg(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace ::BipedalLocomotion::Contacts;

    py::class_<ContactListYarpMsg, ContactList>(module, "ContactListYarpMsg")
        .def(py::init<const ContactList&>())
        .def("from_contact_list", &ContactListYarpMsg::fromContactList, py::arg("contact_list_"))
        .def("__repr__", &ContactListYarpMsg::toString)
        .def("to_string", &ContactListYarpMsg::toString);

    using ::BipedalLocomotion::bindings::YarpUtilities::CreateBufferedPort;
    CreateBufferedPort<ContactListYarpMsg>(module, "BufferedContactList");
}

void CreateContactListMapYarpMsg(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace ::BipedalLocomotion::Contacts;

    py::class_<ContactListMapYarpMsg>(module, "ContactListMapYarpMsg")
        .def(py::init<const ContactListMap&>())
        .def("from_contact_list_map",
             &ContactListMapYarpMsg::fromContactListMap,
             py::arg("contact_list_map"))
        .def("__repr__", &ContactListMapYarpMsg::toString)
        .def("to_string", &ContactListMapYarpMsg::toString);

    using ::BipedalLocomotion::bindings::YarpUtilities::CreateBufferedPort;
    CreateBufferedPort<ContactListMapYarpMsg>(module, "BufferedContactListMap");
}

} // namespace Contacts
} // namespace bindings
} // namespace BipedalLocomotion
