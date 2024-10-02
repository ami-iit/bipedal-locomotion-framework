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
    , m_deadlineMiss(0)
    , m_state(PeriodicThreadState::INACTIVE){};

PeriodicThread::~PeriodicThread()
{
    // stop the thread, if it is not already stopped
    if (m_state.load() != PeriodicThreadState::STOPPED)
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

    m_state.store(PeriodicThreadState::INITIALIZED);

    // synchronize the thread
    synchronize();

    // run loop
    m_state.store(PeriodicThreadState::RUNNING);

    while (m_state == PeriodicThreadState::RUNNING || m_state == PeriodicThreadState::IDLE)
    {
        this->advance();
    }

    m_state.store(PeriodicThreadState::STOPPED);

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

bool PeriodicThread::setPolicy(int priority, int policy)
{
    if (m_state.load() != PeriodicThreadState::INACTIVE)
    {
        BipedalLocomotion::log()->error("[PeriodicThread::setPolicy] The thread has already "
                                        "started. The policy and priority cannot be changed.");
        return false;
    }
    m_priority = priority;
    m_policy = policy;
    return true;
};

bool PeriodicThread::setPeriod(std::chrono::nanoseconds period)
{
    if (m_state.load() != PeriodicThreadState::INACTIVE)
    {
        BipedalLocomotion::log()->error("[PeriodicThread::setPeriod] The thread has already "
                                        "started. The period cannot be changed.");
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
    // stop the thread only if it is running or idling
    if ((m_state.load() == PeriodicThreadState::RUNNING
         || m_state.load() == PeriodicThreadState::IDLE))
    {
        m_state.store(PeriodicThreadState::STOPPED);
    }
};

bool PeriodicThread::suspend()
{
    if (m_state.load() == PeriodicThreadState::RUNNING)
    {
        m_state.store(PeriodicThreadState::IDLE);
        return true;
    }
    return false;
};

bool PeriodicThread::resume()
{
    if (m_state.load() == PeriodicThreadState::IDLE)
    {
        m_state.store(PeriodicThreadState::RUNNING);
        return true;
    }
    return false;
};

bool PeriodicThread::start(std::shared_ptr<BipedalLocomotion::System::Barrier> barrier)
{
    // only an inactive thread can be started
    if (m_state.load() != PeriodicThreadState::INACTIVE)
    {
        return false;
    } else
    {
        m_state.store(PeriodicThreadState::STARTED);
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
    return (m_state.load() == PeriodicThreadState::RUNNING);
}

void PeriodicThread::advance()
{
    // get the current time
    auto now = BipedalLocomotion::clock().now();
    // get the next wake up time
    auto nextWakeUpTime = now + m_period;

    // run user overridden function, when not idling
    if (m_state.load() != PeriodicThreadState::IDLE)
    {
        if (!run())
        {
            // an error occurred, stop the thread
            m_state.store(PeriodicThreadState::STOPPED);
            return;
        }
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
                m_state.store(PeriodicThreadState::STOPPED);
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