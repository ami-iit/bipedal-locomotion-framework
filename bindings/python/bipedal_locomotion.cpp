/**
 * @file bipedal_locomotion.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>

#include "bipedal_locomotion.h"

namespace BipedalLocomotion
{
// Create the Python module
PYBIND11_MODULE(bindings, m)
{
    // QuinticSpline.cpp
    bindings::CreateQuinticSpline(m);

    // ParametersHandler.cpp
    bindings::CreateIParameterHandler(m);
    bindings::CreateStdParameterHandler(m);

    // Contacts.cpp
    bindings::CreateContact(m);
    bindings::CreateContactList(m);
    bindings::CreateContactPhase(m);
    bindings::CreateContactPhaseList(m);
}
} // namespace BipedalLocomotion
