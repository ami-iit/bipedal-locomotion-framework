/**
 * @file CompliantContactWrench.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContactModels/ContactModel.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/CompliantContactWrench.h>

using namespace BipedalLocomotion::ContinuousDynamicalSystem;
using namespace BipedalLocomotion;

CompliantContactWrench::CompliantContactWrench(const iDynTree::FrameIndex& index,
                                               std::shared_ptr<ContactModels::ContactModel> model)

{
    m_contactModel = model;
    m_frame = index;
}

iDynTree::FrameIndex& CompliantContactWrench::index() noexcept
{
    return m_frame;
}

const iDynTree::FrameIndex& CompliantContactWrench::index() const noexcept
{
    return m_frame;
}

const std::weak_ptr<ContactModels::ContactModel>
CompliantContactWrench::contactModel() const noexcept
{
    return m_contactModel;
}
