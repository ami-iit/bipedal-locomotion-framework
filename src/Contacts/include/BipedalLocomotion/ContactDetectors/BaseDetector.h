/**
 * @file BaseDetector.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_BASE_DETECTOR_H
#define BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_BASE_DETECTOR_H

#include <BipedalLocomotion/ContactDetectors/ContactDetector.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

#include <memory>

namespace BipedalLocomotion
{
namespace Contacts
{

/**
 *
 */
class BaseDetector : public ContactDetector
{
    ContactPhaseList m_contactPhaselist;
    double m_currentTime{0};
    double m_dT{0};

    bool
    customInitialization(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) final;

    bool updateContactStates() final;

public:
    void setContactPhaseList(const Contacts::ContactPhaseList& phaseList);
};

} // namespace Contacts
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_BASE_DETECTOR_H
