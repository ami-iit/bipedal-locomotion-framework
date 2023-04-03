/**
 * @file YarpModule.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/Contacts/ContactsYarpMsg.h>
#include <BipedalLocomotion/bindings/Contacts/YarpModule.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Contacts
{
void CreateYarpModule(pybind11::module& module)
{
    CreatePlannedContactYarpMsg(module);
    CreateContactListYarpMsg(module);
    CreateContactListMapYarpMsg(module);
}
} // namespace Contacts
} // namespace bindings
} // namespace BipedalLocomotion
