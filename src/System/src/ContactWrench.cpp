/**
 * @file ContactWrench.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/System/ContactWrench.h>

using namespace BipedalLocomotion::System;

ContactWrench::ContactWrench(const iDynTree::FrameIndex& index)
{
    m_wrench.zero();
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

iDynTree::Wrench& ContactWrench::wrench() noexcept
{
    return m_wrench;
}

const iDynTree::Wrench& ContactWrench::wrench() const noexcept
{
    return m_wrench;
}
