/**
 * @file Barrier.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_BARRIER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_BARRIER_H

#include <condition_variable>
#include <cstddef>
#include <mutex>

namespace BipedalLocomotion
{
namespace System
{
/**
 * Barrier provides a thread-coordination mechanism that allows an expected number of threads to
 * block until the expected number of threads arrive at the barrier.
 */
class Barrier
{
public:
    /**
     * Constructor.
     * @param counter initial value of the expected counter
     */
    explicit Barrier(const std::size_t counter);

    /**
     * Blocks this thread at the phase synchronization point until its phase completion step is run
     */
    void wait();

private:
    std::mutex m_mutex;
    std::condition_variable m_cond;
    std::size_t m_initialCount;
    std::size_t m_count;
    std::size_t m_generation;
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_BARRIER_H
