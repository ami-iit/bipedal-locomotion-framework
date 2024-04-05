/**
 * @file CompliantContactWrench.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_COMPLIANT_CONTACT_WRENCH_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_COMPLIANT_CONTACT_WRENCH_H

#include <memory>

#include <BipedalLocomotion/ContactModels/ContactModel.h>
#include <iDynTree/Indices.h>

namespace BipedalLocomotion
{

namespace ContinuousDynamicalSystem
{

/**
 * A wrench excerted on a link due to an external contact.
 */
class CompliantContactWrench
{
    iDynTree::FrameIndex m_frame; /**< Useful for identifying the variable in the Model */
    std::shared_ptr<ContactModels::ContactModel> m_contactModel; /**< Contact model */

public:
    /**
     * Constructor
     * @param index index of the frame
     * @param wrench the contact wrench
     */
    CompliantContactWrench(const iDynTree::FrameIndex& index,
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
    const std::weak_ptr<ContactModels::ContactModel> contactModel() const noexcept;
};

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_COMPLIANT_CONTACT_WRENCH_H
