/**
 * @file Contacts.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_CONTACTS_CONTACTS_H
#define BIPEDAL_LOCOMOTION_BINDINGS_CONTACTS_CONTACTS_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Contacts
{

void CreateContact(pybind11::module& module);
void CreateContactList(pybind11::module& module);
void CreateContactPhase(pybind11::module& module);
void CreateContactPhaseList(pybind11::module& module);
void CreateContactListJsonParser(pybind11::module& module);

} // namespace Contacts
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_CONTACTS_CONTACTS_H
