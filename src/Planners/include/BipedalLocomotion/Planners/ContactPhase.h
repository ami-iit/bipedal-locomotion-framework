/**
 * @file ContactPhase.h
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_PHASE_H
#define BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_PHASE_H

#include <BipedalLocomotion/Planners/Contact.h>
#include <BipedalLocomotion/Planners/ContactList.h>
#include <vector>

namespace BipedalLocomotion
{
namespace Planners
{

struct ContactPhase
{
    double begin {0.0};

    double end {0.0};

    std::vector<ContactList::const_iterator> activeContacts;
};

}
}

#endif // BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_PHASE_H
