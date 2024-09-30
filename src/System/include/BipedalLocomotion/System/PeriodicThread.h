/**
 * @file PeriodicThread.h
 * @authors Lorenzo Moretti
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_PERIODIC_THREAD_H
#define BIPEDAL_LOCOMOTION_SYSTEM_PERIODIC_THREAD_H

#include <atomic>
#include <chrono>
#include <thread>

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
    virtual ~PeriodicThread();

    // Copy constructor
    PeriodicThread(const PeriodicThread&) = delete;

    // Copy assignment operator
    PeriodicThread& operator=(const PeriodicThread&) = delete;

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

    /**
     * @brief Set the period of the thread.
     * @param period period of the thread.
     * @return true if the period was correctly set, false otherwise.
     */
    bool setPeriod(std::chrono::nanoseconds period);

protected:
    /**
     * @brief This method is called at each iteration of the thread.
     * Override this method to implement the task to be performed from the thread.
     * @return true if the thread has to continue, false otherwise.
     */
    virtual bool run() = 0;

    /**
     * @brief This method is called at the beginning of the thread.
     * @return true if the initialization was successful, false otherwise.
     */
    virtual bool threadInit();

private:
    /**
     * @brief run the periodic thread.
     */
    void threadFunction();

    /**
     * @brief Advance the thread of one step, calling the user defined run function once.
     */
    void advance();

    /**
     * @brief Set the policy of the thread.
     * @return true if the policy was correctly set, false otherwise.
     */
    bool setPolicy();

    std::chrono::nanoseconds m_period = std::chrono::nanoseconds(100000); /**< Period of the thread.
                                                                           */

    int m_maximumNumberOfAcceptedDeadlineMiss = -1; /**< Maximum number of accepted deadline miss.
                                                     */

    int m_deadlineMiss = 0; /**< Number of deadline miss. */

    int m_priority = 0; /**< Priority of the thread. */

    int m_policy = SCHED_OTHER; /**< Policy of the thread. */

    std::atomic<bool> m_isRunning = false; /**< Flag to check if the thread is running. */

    std::atomic<bool> m_askToStop = false; /**< Flag to ask to stop the thread. */

    std::thread m_thread; /**< Thread object. */
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_PERIODIC_THREAD_H