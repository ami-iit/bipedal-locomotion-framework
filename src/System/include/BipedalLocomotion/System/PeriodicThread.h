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
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <BipedalLocomotion/System/Barrier.h>

namespace BipedalLocomotion
{
namespace System
{

/**
 * @brief Enum class which defines the state of the periodic thread state machine.
 */
enum class PeriodicThreadState
{
    INACTIVE, /**< The thread is not yet active. */
    STARTED, /**< The thread has been started. */
    INITIALIZED, /**< The thread has been initialized.*/
    EXECUTING, /**< The thread is executing the task. */
    IDLE, /**< The thread is idling. */
    STOPPED /**< The thread is stopped. */
};

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
                   int policy = SCHED_OTHER,
                   bool earlyWakeUp = false);

    // Destructor
    virtual ~PeriodicThread();

    // Copy constructor
    PeriodicThread(const PeriodicThread&) = delete;

    // Copy assignment operator
    PeriodicThread& operator=(const PeriodicThread&) = delete;

    /**
     * @brief Start the thread.
     * @param barrier barrier to synchronize the thread. If nullptr, the thread will start
     * immediately, without waiting for other threads to reach the barrier.
     * @return true if the thread was correctly started, false otherwise.
     */
    bool start(std::shared_ptr<BipedalLocomotion::System::Barrier> barrier = nullptr);

    /**
     * @brief Call this method to stop the thread.
     */
    void stop();

    /**
     * @brief Suspend the thread.
     */
    bool suspend();

    /**
     * @brief Resume the thread.
     */
    bool resume();

    /**
     * @brief Check if the thread is running. This means that the thread has started and not yet
     * stopped.
     * @return true if the thread is running, false otherwise.
     */
    bool isRunning();

    /**
     * @brief Check if the thread is initialized.
     * @return true if the thread is initialized, false otherwise.
     */
    bool isInitialized();

    /**
     * @brief Set the period of the thread.
     * @param period period of the thread.
     * @return true if the period was correctly set, false otherwise.
     */
    bool setPeriod(std::chrono::nanoseconds period);

    /**
     * @brief Set the policy of the thread.
     * @param policy policy of the thread.
     * @param priority priority of the thread.
     * @return true if the policy was correctly set, false otherwise.
     */
    bool setPolicy(int policy, int priority = 0);

    /**
     * @brief Set the maximum number of accepted deadline miss.
     * @param maximumNumberOfAcceptedDeadlineMiss maximum number of accepted deadline miss.
     */
    bool setMaximumNumberOfAcceptedDeadlineMiss(int maximumNumberOfAcceptedDeadlineMiss);

    /**
     * @brief Get the number of deadline miss.
     * @return number of deadline miss.
     */
    int getNumberOfDeadlineMiss();

    /**
     * @brief Enable the early wake up. The thread will be awaken before and busy wait until the
     * actual wake up time.
     * @return true if the early wake up was correctly set, false otherwise.
     */
    bool enableEarlyWakeUp();

protected:
    /**
     * @brief This method is called at each iteration of the thread.
     * Override this method to implement the task to be performed from the thread.
     * @return true if the thread has to continue, false otherwise.
     */
    virtual bool run();

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
     * @brief Synchronize the thread.
     */
    void synchronize();

    /**
     * @brief Set the policy of the thread.
     * @return true if the policy was correctly set, false otherwise.
     */
    bool setPolicy();

    std::chrono::nanoseconds m_period = std::chrono::nanoseconds(100000); /**< Period of the
                                                                           * thread.
                                                                           */

    int m_maximumNumberOfAcceptedDeadlineMiss = -1; /**< Maximum number of accepted deadline
                                                     * miss.
                                                     */

    std::atomic<int> m_deadlineMiss = 0; /**< Number of deadline miss. */

    std::chrono::nanoseconds m_wakeUpTime = std::chrono::nanoseconds(0); /**< Wake up time of the
                                                                          * thread.
                                                                          */
    int m_priority = 0; /**< Priority of the thread. */

    int m_policy = SCHED_OTHER; /**< Policy of the thread. */

    std::thread m_thread; /**< Thread object. */

    std::atomic<PeriodicThreadState> m_state = PeriodicThreadState::INACTIVE; /**< State of the
                                                                               * thread.
                                                                               */

    std::shared_ptr<BipedalLocomotion::System::Barrier> m_barrier; /**< Barrier to synchronize the
                                                                    * thread.
                                                                    */
    const std::chrono::nanoseconds m_schedulerLatency
        = std::chrono::microseconds(100); /**< Scheduler latency when waking up a thread. Indeed, it
                                             varies depending on the OS. For now we fix it to a
                                             constant value. Note that in real-time OS it might be
                                             smaller. */
    bool m_earlyWakeUp = false; /**< If true, the thread will be awaken before and busy wait until
                                   the actual wake up time. */
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_PERIODIC_THREAD_H