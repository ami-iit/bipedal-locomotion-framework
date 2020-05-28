/**
 * @file ContactWrench.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_CONTACT_WRENCH_H
#define BIPEDAL_LOCOMOTION_SYSTEM_CONTACT_WRENCH_H

#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Model/Indices.h>
#include <BipedalLocomotion/ContactModels/ContactModel.h>

namespace BipedalLocomotion
{

namespace System
{

/**
 * A wrench excerted on a link due to an external contact.
 */
class ContactWrench
{
    iDynTree::FrameIndex m_frame; /**< Useful for identifying the variable in the Model */
    std::shared_ptr<ContactModels::ContactModel> m_contactModel; /**< Contact model */

public:

    /**
     * Constructor
     * @param index index of the frame
     * @param wrench the contact wrench
     */
    ContactWrench(const iDynTree::FrameIndex& index,
                  std::shared_ptr<ContactModels::ContactModel> model);

    /**
     * Get the index of the frame
     * @return a reference to the index of the frame
     */
    iDynTree::FrameIndex& index() noexcept;

    /**
     * Get the index of the frame
     * @return a const reference to the index of the frame
     */
    const iDynTree::FrameIndex& index() const noexcept;

    /**
     * Get the contact wrench
     * @return a const reference to the parameter
     */
    const std::shared_ptr<ContactModels::ContactModel>& contactModel() const noexcept;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_CONTACT_WRENCH_H
