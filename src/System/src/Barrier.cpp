/**
 * @file Barrier.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <mutex>

#include <BipedalLocomotion/System/Barrier.h>

using namespace BipedalLocomotion::System;

Barrier::Barrier(const std::size_t counter)
    : m_initialCount(counter)
    , m_count(counter)
    , m_generation(0)
{
}

void Barrier::wait()
{
    std::unique_lock lock{m_mutex};
    const auto tempGeneration = m_generation;
    if ((--m_count) == 1)
    {
        // all threads reached the barrier, so we can consider them synchronized
        m_generation++;

        // reset the counter
        m_count = m_initialCount;

        // notify the other threads
        m_cond.notify_all();
    } else
    {
        // waiting for the other threads
        m_cond.wait(lock, [this, tempGeneration] { return tempGeneration != m_generation; });
    }
}
