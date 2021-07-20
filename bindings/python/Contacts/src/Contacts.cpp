/**
 * @file Contacts.cpp
 * @authors Diego Ferigo, Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Contacts/ContactPhase.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/Contacts/ContactListJsonParser.h>

#include <BipedalLocomotion/bindings/Contacts/Contacts.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Contacts
{

std::string toString(const BipedalLocomotion::Contacts::PlannedContact& contact)
{
    std::string description;

    const Eigen::IOFormat FormatEigenVector //
        (Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

    const auto& position = contact.pose.coeffs().segment<3>(0);
    const auto& quaternion = contact.pose.coeffs().segment<4>(3);

    std::stringstream pose;
    pose << "SE3 (position = " << position.format(FormatEigenVector)
         << ", quaternion = " << quaternion.format(FormatEigenVector) << ")";

    description = "Contact (name = " + contact.name + ", pose = " + pose.str()
                  + ", activation_time = " + std::to_string(contact.activationTime)
                  + ", deactivation_time = " + std::to_string(contact.deactivationTime)
                  + ", type = " + std::to_string(static_cast<int>(contact.type)) + ")";

    return description;
}

void CreateContact(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Contacts;

    py::enum_<ContactType>(module, "ContactType")
        .value("Full", ContactType::FULL)
        .value("Point", ContactType::POINT);

    py::class_<ContactBase>(module, "ContactBase")
        .def(py::init())
        .def_readwrite("pose", &ContactBase::pose)
        .def_readwrite("index", &ContactBase::index)
        .def_readwrite("name", &PlannedContact::name)
        .def_readwrite("type", &ContactBase::type);

    py::class_<PlannedContact, ContactBase>(module, "PlannedContact")
        .def(py::init())
        .def_readwrite("activation_time", &PlannedContact::activationTime)
        .def_readwrite("deactivation_time", &PlannedContact::deactivationTime)
        .def("__repr__", &toString)
        .def("__eq__", &PlannedContact::operator==, py::is_operator());

    py::class_<EstimatedContact, ContactBase>(module, "EstimatedContact")
        .def(py::init())
        .def_readwrite("switch_time", &EstimatedContact::switchTime)
        .def_readwrite("is_active", &EstimatedContact::isActive)
        .def_readwrite("last_update_time", &EstimatedContact::lastUpdateTime)
        .def("get_contact_details", &EstimatedContact::getContactDetails)
        .def("set_contact_state_stamped", &EstimatedContact::setContactStateStamped);
}

void CreateContactList(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Contacts;

    py::class_<ContactList>(module, "ContactList")
        .def(py::init())
        .def("set_default_name", &ContactList::setDefaultName, py::arg("default_name"))
        .def("default_name", &ContactList::defaultName)
        .def("set_default_contact_type", &ContactList::setDefaultContactType, py::arg("type"))
        .def("default_contact_type", &ContactList::defaultContactType)
        .def("add_contact",
             py::overload_cast<const PlannedContact&>(&ContactList::addContact),
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
            [](const ContactList& l, const double time) -> PlannedContact {
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

void CreateContactPhase(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Contacts;

    py::class_<ContactPhase>(module, "ContactPhase")
        .def(py::init())
        .def_readwrite("begin_time", &ContactPhase::beginTime)
        .def_readwrite("end_time", &ContactPhase::endTime)
        .def_property_readonly("active_contacts",
                               [](const ContactPhase& phase)
                                   -> std::unordered_map<std::string, PlannedContact> {
                                   std::unordered_map<std::string, PlannedContact> map;

                                   for (const auto& [key, iter] : phase.activeContacts)
                                   {
                                       map[key] = *iter;
                                   }

                                   return map;
                               })
        .def("is_list_included", &ContactPhase::isListIncluded, py::arg("key"));
}

void CreateContactPhaseList(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Contacts;
    py::class_<ContactPhaseList>(module, "ContactPhaseList")
        .def(py::init())
        .def("set_lists",
             py::overload_cast<const ContactListMap&>(&ContactPhaseList::setLists),
             py::arg("contact_lists"))
        .def("lists", &ContactPhaseList::lists);
}

void CreateContactListJsonParser(pybind11::module& module)
{
    namespace py = ::pybind11;

    module.def("contact_list_map_from_json",
               BipedalLocomotion::Contacts::contactListMapFromJson,
               py::arg("filename"));
    module.def("contact_list_map_to_json",
               BipedalLocomotion::Contacts::contactListMapToJson,
               py::arg("map"),
               py::arg("filename"));
}

} // namespace contatcs
} // namespace bindings
} // namespace BipedalLocomotion
