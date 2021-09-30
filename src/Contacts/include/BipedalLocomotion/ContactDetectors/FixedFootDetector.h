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
 * The logic used to find the fixed foot is the follow:
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
 * @warning The FixedFootDetector class can handle only bipedal locomotion. I.e. the
 * ContactPhaseList passed to FixedFootDetector::setContactPhaseList must contain only single
 * support and double phases.
 * @warning The presented logic may not fit in a real scenario. Indeed the class implements an
 * approach completely based on the desired footsteps and it does not consider the current state of
 * the robot. We suggest to use Contacts::SchmittTriggerDetector class if you want consider the
 * force acting on the robot.
 * @note Given the current implementation of the class, you should call `ContactDetector::advance`
 * only to `advance` the current time stored within the class. If you want to get the fixed foot at
 * the initial time \f$t=t_i\f$, you should call `FixedFootDetector::getFixedFoot` before calling
 * advance. The following snippet can be used as a reference:
 * @code{.cpp}
 * FixedFootDetector detector;
 *
 * // initialize the detector
 * detector.initialize(paramsHandler);
 * detector.setContactPhaseList(contactPhaseList);
 *
 * // get the fixed frame at initial time instant (t = t_i)
 * auto fixedFoot = detector.getFixedFoot();
 *
 * // get the fixed frame at initial time + sampling time instant (t = t_i + dt)
 * detector.advance();
 * auto fixedFoot = detector.getFixedFoot();
 * @endcode
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
     * |   Parameter Name  |    Type    |                Description                 | Mandatory |
     * |:-----------------:|:----------:|:------------------------------------------:|:---------:|
     * |  `sampling_time`  |  `double`  |  Sampling time of the detector is seconds  |    Yes    |
     * @return true in case of success/false otherwise.
     */
    bool customInitialization(std::weak_ptr<const //
                                            ParametersHandler::IParametersHandler> handler) final;

    /**
     * Update the contact state. This function advance the current time stored in the class.
     * @return true in case of success/false otherwise.
     */
    bool updateContactStates() final;

    /**
     * Update the fixed foot.
     * @return true in case of success/false otherwise.
     */
    bool updateFixedFoot();

    /**
     * Set the internal buffer related to the ContactPhaseList
     * @param phaseList a contact phase list
     * @note this method sets only the internal buffer used by the detector
     */
    void setContactPhaseListPrivate(const ContactPhaseList& phaseList);

public:

    /**
     * Set the contact phase list
     * @param phaseList a contact phase list
     */
    bool setContactPhaseList(const ContactPhaseList& phaseList);

    /**
     * Set the contact phase list
     * @param phaseList a contact phase list
     * @warning this method does not update the m_currentTime variable. It should be used only if
     * the setContactPhaseList method has been called at least once. Here we may have some
     * discontinuity in the estimated contact. Please use it only if you are confident on what you
     * are doing.
     */
    bool setContactPhaseListWithoutResetInternalTime(const ContactPhaseList& phaseList);

    /**
     * Get the fixed foot
     * @return the fixed foot.
     */
    const EstimatedContact& getFixedFoot() const;
};

} // namespace Contacts
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_FIXED_FOOT_DETECTOR_H
