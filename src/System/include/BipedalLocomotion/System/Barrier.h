/**
 * @file Barrier.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
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
     * Calls the constructor. It creates a new Barrier with the given counter.
     */
    static std::shared_ptr<Barrier> create(const std::size_t counter);

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

    /**
     * Constructor.
     * @param counter initial value of the expected counter
     */
    explicit Barrier(const std::size_t counter);
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_BARRIER_H
