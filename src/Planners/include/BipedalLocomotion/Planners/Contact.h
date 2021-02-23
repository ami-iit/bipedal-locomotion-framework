/**
 * @file Contact.h
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_H
#define BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_H

#include <BipedalLocomotion/Contacts/Contact.h>


#include <string>

namespace BipedalLocomotion
{
namespace Planners
{

[[deprecated("BipedalLocomotion::Planners::ContactType is deprecated; use "
             "BipedalLocomotion::Contacts::ContactType instead.")]]
typedef BipedalLocomotion::Contacts::ContactType ContactType;

[[deprecated("BipedalLocomotion::Planners::Contact is deprecated; use "
             "BipedalLocomotion::Contacts::PlannedContact instead.")]]
typedef BipedalLocomotion::Contacts::PlannedContact Contact;

}
}

#endif // BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_H
