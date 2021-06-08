/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/Contacts/ContactDetectors.h>
#include <BipedalLocomotion/bindings/Contacts/Contacts.h>
#include <BipedalLocomotion/bindings/Contacts/Module.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Contacts
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "Contacts module.";

    CreateContact(module);
    CreateContactList(module);
    CreateContactPhase(module);
    CreateContactPhaseList(module);
    CreateContactListJsonParser(module);

    CreateContactDetector(module);
    CreateSchmittTriggerUnit(module);
    CreateSchmittTriggerDetector(module);
    CreateFixedFootDetector(module);
}
} // namespace Contacts
} // namespace bindings
} // namespace BipedalLocomotion
