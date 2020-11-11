/**
 * @file Contacts.cpp
 * @authors Diego Ferigo
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "BipedalLocomotion/Planners/Contact.h"
#include "BipedalLocomotion/Planners/ContactList.h"
#include "BipedalLocomotion/Planners/ContactPhase.h"
#include "BipedalLocomotion/Planners/ContactPhaseList.h"
#include "bipedal_locomotion.h"

void BipedalLocomotion::bindings::CreateContact(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Planners;

    py::enum_<ContactType>(module, "ContactType")
        .value("Full", ContactType::FULL)
        .value("Point", ContactType::POINT);

    py::class_<Contact>(module, "Contact")
        .def(py::init())
        .def_readwrite("pose", &Contact::pose)
        .def_readwrite("activation_time", &Contact::activationTime)
        .def_readwrite("deactivation_time", &Contact::deactivationTime)
        .def_readwrite("name", &Contact::name)
        .def_readwrite("type", &Contact::type)
        .def_readwrite("pose", &Contact::pose)
        .def("__repr__", py::overload_cast<const Contact&>(&ToString))
        .def("__eq__", &Contact::operator==, py::is_operator());
}

void BipedalLocomotion::bindings::CreateContactList(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Planners;

    py::class_<ContactList>(module, "ContactList")
        .def(py::init())
        .def("set_default_name", &ContactList::setDefaultName, py::arg("default_name"))
        .def("default_name", &ContactList::defaultName)
        .def("set_default_contact_type", &ContactList::setDefaultContactType, py::arg("type"))
        .def("default_contact_type", &ContactList::defaultContactType)
        .def("add_contact",
             py::overload_cast<const Contact&>(&ContactList::addContact),
             py::arg("contact"))
        .def("add_contact",
             py::overload_cast<const manif::SE3d&, double, double>(&ContactList::addContact),
             py::arg("transform"),
             py::arg("activation_time"),
             py::arg("deactivation_time"))
        .def(
            "__iter__",
            [](const ContactList& l) { return py::make_iterator(l.cbegin(), l.cend()); },
            py::keep_alive<0, 1>())
        .def("__getitem__", &ContactList::operator[])
        .def("__getitem__",
             [](const ContactList& l, py::slice slice) -> ContactList* {
                 size_t start, stop, step, sliceLength;
                 if (!slice.compute(l.size(), &start, &stop, &step, &sliceLength))
                     throw py::error_already_set();
                 auto* slicedList = new ContactList();
                 for (size_t i = 0; i < sliceLength; ++i)
                 {
                     slicedList->addContact(l[start]);
                     start += step;
                 }
                 return slicedList;
             })
        .def("size", &ContactList::size)
        .def("__len__", &ContactList::size)
        .def("edit_contact", &ContactList::editContact, py::arg("element"), py::arg("new_contact"))
        .def(
            "get_present_contact",
            [](const ContactList& l, const double time) -> Contact {
                return *l.getPresentContact(time);
            },
            py::arg("time"))
        .def("keep_only_present_contact", &ContactList::keepOnlyPresentContact, py::arg("time"))
        .def("clear", &ContactList::clear)
        .def("remove_last_contact", &ContactList::removeLastContact)
        .def("__repr__",
             [](const ContactList& list) {
                 return "ContactList(" + std::to_string(list.size()) + ")";
             })
        .def("__reverse__",
             [](const ContactList& l) { return py::make_iterator(l.crbegin(), l.crend()); });
}

void BipedalLocomotion::bindings::CreateContactPhase(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Planners;

    py::class_<ContactPhase>(module, "ContactPhase")
        .def(py::init())
        .def_readwrite("begin_time", &ContactPhase::beginTime)
        .def_readwrite("end_time", &ContactPhase::endTime)
        .def_property_readonly("active_contacts",
                               [](const ContactPhase& phase)
                                   -> std::unordered_map<std::string, Contact> {
                                   std::unordered_map<std::string, Contact> map;

                                   for (const auto& [key, iter] : phase.activeContacts)
                                   {
                                       map[key] = *iter;
                                   }

                                   return map;
                               })
        .def("is_list_included", &ContactPhase::isListIncluded, py::arg("key"));
}

void BipedalLocomotion::bindings::CreateContactPhaseList(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Planners;
    py::class_<ContactPhaseList, std::shared_ptr<ContactPhaseList>>(module, "ContactPhaseList")
        .def(py::init())
        .def("set_lists",
             py::overload_cast<const ContactListMap&>(&ContactPhaseList::setLists),
             py::arg("contact_lists"))
        .def("lists", &ContactPhaseList::lists);
}
