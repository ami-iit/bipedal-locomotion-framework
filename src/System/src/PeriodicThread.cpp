#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
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
    // stop the thread, if it is running
    if (m_isRunning.load())
    {
        stop();
    }

    // join the thread
    if (m_thread.joinable())
    {
        m_thread.join();
        m_thread = std::thread();
        BipedalLocomotion::log()->info("[PeriodicThread::~PeriodicThread] The thread joined.");
    }

    //
    std::cerr << "[PeriodicThread::~PeriodicThread] The thread object is "
                 "destroyed."
              << std::endl;
};

void PeriodicThread::threadFunction()
{
    constexpr auto logPrefix = "[PeriodicThread::threadFunction]";

    // thread initialization
    if (!threadInit())
    {
        return;
    }

    // set the policy
    if (!setPolicy())
    {
        return;
    }

    // synchronize the thread
    synchronize();

    // run loop
    while (m_isRunning.load() && !m_askToStop.load())
    {
        this->advance();
    }

    m_isRunning.store(false);

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

bool PeriodicThread::setPeriod(std::chrono::nanoseconds period)
{
    if (m_isRunning.load())
    {
        BipedalLocomotion::log()->error("[PeriodicThread::setPeriod] The thread is running. The "
                                        "period cannot be changed.");
        return false;
    }
    m_period = period;
    return true;
}

bool PeriodicThread::threadInit()
{
    return true;
};

void PeriodicThread::synchronize()
{
    if (!(m_barrier == nullptr))
    {
        BipedalLocomotion::log()->debug("[PeriodicThread::synchronize] This thread is waiting for "
                                        "the other threads.");
        m_barrier->wait();
    }
};

void PeriodicThread::stop()
{
    if (!m_isRunning.load())
    {
        // thread is not running
        return;
    }
    if (m_askToStop.load())
    {
        // thread is already asked to stop
        return;
    }
    m_askToStop.store(true);
};

bool PeriodicThread::start(std::shared_ptr<BipedalLocomotion::System::Barrier> barrier)
{

    if (m_isRunning.load())
    {
        // thread is already running
        return false;
    } else
    {
        m_isRunning.store(true);
    }

    // store the barrier
    m_barrier = barrier;

    // lambda wrapper for the thread function
    auto threadFunctionLambda = [this]() { this->threadFunction(); };

    m_thread = std::thread(threadFunctionLambda);

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