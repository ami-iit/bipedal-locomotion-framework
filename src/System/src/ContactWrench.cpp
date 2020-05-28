/**
 * @file ContactWrench.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/System/ContactWrench.h>

using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion;

ContactWrench::ContactWrench(const iDynTree::FrameIndex& index,
                             std::shared_ptr<ContactModels::ContactModel> model)

{
    m_contactModel = model;
    m_frame = index;
}

iDynTree::FrameIndex& ContactWrench::index() noexcept
{
    return m_frame;
}

const iDynTree::FrameIndex& ContactWrench::index() const noexcept
{
    return m_frame;
}

const std::shared_ptr<ContactModels::ContactModel>& ContactWrench::contactModel() const noexcept
{
    return m_contactModel;
}
