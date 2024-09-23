/**
 * @file PeriodicThread.h
 * @authors Lorenzo Moretti
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_PERIODIC_THREAD_H
#define BIPEDAL_LOCOMOTION_SYSTEM_PERIODIC_THREAD_H

#include <chrono>
#include <memory>

namespace BipedalLocomotion
{
namespace System
{
/**
 * @brief This class implements a periodic thread. The user has to inherit from this class and
 * implement the virtual methods.
 */

class PeriodicThread
{
public:
    // Default constructor
    PeriodicThread(std::chrono::nanoseconds period = std::chrono::nanoseconds(100000),
                   int maximumNumberOfAcceptedDeadlineMiss = -1,
                   int priority = 0,
                   int policy = SCHED_OTHER);

    // Destructor
    ~PeriodicThread();

    /**
     * @brief This method is called at each iteration of the thread.
     * Override this method to implement the thread itself.
     * @return true if the thread has to continue, false otherwise.
     */
    virtual bool run() = 0;

    /**
     * @brief This method is called at the beginning of the thread.
     * @return true if the initialization was successful, false otherwise.
     */
    virtual bool threadInit();

    /**
     * @brief Start the thread
     * @return true if the thread was correctly started, false otherwise.
     */
    bool start();

    /**
     * @brief Call this method to stop the thread.
     */
    void stop();

    /**
     * @brief Check if the thread is running.
     * @return true if the thread is running, false otherwise.
     */
    bool isRunning();

private:
    // private implementation
    class Impl;
    Impl* m_impl;
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_PERIODIC_THREAD_H