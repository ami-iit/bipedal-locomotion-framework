/**
 * @file FixedFootDetector.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_FIXED_FOOT_DETECTOR_H
#define BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_FIXED_FOOT_DETECTOR_H

#include <BipedalLocomotion/ContactDetectors/ContactDetector.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

#include <memory>

namespace BipedalLocomotion
{
namespace Contacts
{

/**
 * The FixedFootDetector is a class that can be used to find the fixed foot given a sequence of
 * contacts.
 * The logic used to find is the follows:
 * - In the single support phase the fixed foot is the stance foot
 * - In the double support phase the fixed foot is the foot in switch off phase, where switch off is
 *   the transition phase between stance and swing.
 *
 * The aforementioned logic is described by the following diagram
 * @code{.unparsed}
 * time         0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17
 * L            |+++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|
 * R            |+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++|
 * stance foot  |LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|
 * @endcode
 * @warning The FixedFootDetector can handle only bipedal locomotion. Furthermore only one foot is
 * considered as fixed foot
 * @warning The presented logic may not fit in a real scenario. Indeed the class implements an
 * approach completely based on the desired value and it does not consider the current state of the
 * robot. We suggest to use Contacts::SchmittTriggerDetector class if you want consider the force
 * acting on the robot.
 */
class FixedFootDetector : public ContactDetector
{
    ContactPhaseList m_contactPhaselist; /**< List of the contacts */
    double m_currentTime{0}; /**< Current time in seconds */
    double m_dT{0}; /**< Fixed sampling time in seconds */
    EstimatedContact m_dummyContact; /**< A dummy esitmated contact */

    /**
     * Initialize the detector.
     * @param handler pointer to the parameter handler.
     * @note the following parameters are required by the class
     * |   Parameter Name  |    Type    |                  Description                   | Mandatory |
     * |:-----------------:|:----------:|:----------------------------------------------:|:---------:|
     * |  `sampling_time`  |  `double`  |    Sampling time of the detector is seconds    |    Yes    |
     * @return true in case of success/false otherwise.
     */
    bool customInitialization(std::weak_ptr<const //
                                            ParametersHandler::IParametersHandler> handler) final;

    /**
     * Update the contact state.
     * @return true in case of success/false otherwise.
     */
    bool updateContactStates() final;

public:

    /**
     * Set the contact phase list
     * @param phaseList a contact phase list
     */
    void setContactPhaseList(const ContactPhaseList& phaseList);

    /**
     * Get the fixed foot
     * @return the fixed foot.
     */
    const EstimatedContact& getFixedFoot() const;
};

} // namespace Contacts
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_FIXED_FOOT_DETECTOR_H
