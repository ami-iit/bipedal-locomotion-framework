#include <atomic>
#include <chrono>
#include <csignal>
#include <pthread.h>
#include <sched.h>

#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/PeriodicThread.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
namespace System
{

PeriodicThread::PeriodicThread(std::chrono::nanoseconds period,
                               int maximumNumberOfAcceptedDeadlineMiss,
                               int priority,
                               int policy)
    : m_period(period)
    , m_maximumNumberOfAcceptedDeadlineMiss(maximumNumberOfAcceptedDeadlineMiss)
    , m_priority(priority)
    , m_policy(policy)
    , m_deadlineMiss(0){};

PeriodicThread::~PeriodicThread()
{
    // join the thread
    if (m_thread.joinable())
    {
        m_thread.join();
    }
};

void PeriodicThread::threadFunction()
{
    constexpr auto logPrefix = "[PeriodicThread::threadFunction]";

    BipedalLocomotion::log()->error("{} about to initialize the thread", logPrefix);

    // thread initialization
    if (!threadInit())
    {
        BipedalLocomotion::log()->error("{} Failed to initialize the thread", logPrefix);
        return;
    }

    BipedalLocomotion::log()->error("{} thread initialized", logPrefix);

    // set the policy
    if (!setPolicy())
    {
        BipedalLocomotion::log()->error("{} Failed to set the policy", logPrefix);
        return;
    }

    BipedalLocomotion::log()->error("{} policy set", logPrefix);

    // run loop
    BipedalLocomotion::log()->error("{} advance... {}", logPrefix, m_isRunning.load());
    while (m_isRunning.load())
    {
        this->advance();
    }

    m_isRunning.store(false);
    BipedalLocomotion::log()->error("{} Completing... {}", logPrefix, m_isRunning.load());

    return;
};

bool PeriodicThread::setPolicy()
{
#ifdef __linux__
    // get the current thread native handle
    pthread_t nativeHandle = pthread_self();
    // get the current thread parameters
    sched_param params;
    params.sched_priority = m_priority;
    // set the new policy
    return (pthread_setschedparam(nativeHandle, m_policy, &params) == 0);
#else
    return true;
#endif
};

bool PeriodicThread::run()
{
    BipedalLocomotion::log()->info("[PeriodicThread::run] Base Class.");
    return true;
};

bool PeriodicThread::threadInit()
{
    return true;
};

void PeriodicThread::stop()
{
    m_askToStop.store(true);
};

bool PeriodicThread::start()
{
    BipedalLocomotion::log()->info("[PeriodicThread::start] Thread is already running: {}",
                                   m_isRunning.load());

    if (m_isRunning.load())
    {
        // thread is already running
        BipedalLocomotion::log()->error("[PeriodicThread::start] The thread is already "
                                        "running.");
        return false;
    } else
    {
        m_isRunning.store(true);
    }

    BipedalLocomotion::log()->info("[PeriodicThread::start] Thread is running: {}",
                                   m_isRunning.load());

    // lambda wrapper for the thread function
    auto threadFunctionLambda = [this]() { this->threadFunction(); };

    m_thread = std::thread(threadFunctionLambda);

    BipedalLocomotion::log()->info("[PeriodicThread::start] Thread runs");

    return m_thread.joinable();
};

bool PeriodicThread::isRunning()
{
    return m_isRunning.load();
}

void PeriodicThread::advance()
{
    // get the current time
    auto now = BipedalLocomotion::clock().now();
    // get the next wake up time
    auto nextWakeUpTime = now + m_period;

    // run user overridden function
    if (!run())
    {
        BipedalLocomotion::log()->info("[PeriodicThread::advance] Run failed");
        m_isRunning.store(false);
        return;
    }

    // check if the thread has to stop
    if (m_askToStop.load())
    {
        BipedalLocomotion::log()->info("[PeriodicThread::advance] Asked to stop");
        m_isRunning.store(false);
        return;
    }

    // check if the deadline is missed
    if (BipedalLocomotion::clock().now() > nextWakeUpTime)
    {
        m_deadlineMiss++;
        if (m_maximumNumberOfAcceptedDeadlineMiss > 0)
        {
            if (m_deadlineMiss > m_maximumNumberOfAcceptedDeadlineMiss)
            {
                // we have to close the runner
                BipedalLocomotion::log()->info("[PeriodicThread::advance] Deadline missed");
                m_isRunning.store(false);
                return;
            }
        }
    }

    // yield the CPU
    BipedalLocomotion::clock().yield();

    // wait until the next deadline
    BipedalLocomotion::clock().sleepUntil(nextWakeUpTime);
};

} // namespace System
} // namespace BipedalLocomotion