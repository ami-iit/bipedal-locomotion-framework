#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <thread>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/PeriodicThread.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
namespace System
{

#ifdef __linux__

PeriodicThread::PeriodicThread(std::chrono::nanoseconds period,
                               int maximumNumberOfAcceptedDeadlineMiss,
                               int priority,
                               int policy,
                               bool earlyWakeUp)
    : m_period(period)
    , m_maximumNumberOfAcceptedDeadlineMiss(maximumNumberOfAcceptedDeadlineMiss)
    , m_priority(priority)
    , m_policy(policy)
    , m_deadlineMiss(0)
    , m_wakeUpTime(std::chrono::nanoseconds(0))
    , m_earlyWakeUp(earlyWakeUp)
    , m_state(PeriodicThreadState::INACTIVE){};

#else

PeriodicThread::PeriodicThread(std::chrono::nanoseconds period,
                               int maximumNumberOfAcceptedDeadlineMiss,
                               bool earlyWakeUp)
    : m_period(period)
    , m_maximumNumberOfAcceptedDeadlineMiss(maximumNumberOfAcceptedDeadlineMiss)
    , m_deadlineMiss(0)
    , m_wakeUpTime(std::chrono::nanoseconds(0))
    , m_earlyWakeUp(earlyWakeUp)
    , m_state(PeriodicThreadState::INACTIVE){};

#endif

PeriodicThread::~PeriodicThread()
{
    // force stop of the thread
    m_state.store(PeriodicThreadState::STOPPED);

    // join the thread
    if (m_thread.joinable())
    {
        m_thread.join();
        m_thread = std::thread();
    }
};

void PeriodicThread::threadFunction()
{
    constexpr auto logPrefix = "[PeriodicThread::threadFunction]";

    m_initializationSuccessful.store(true);

    // thread initialization
    if (!threadInit())
    {
        log()->error("{} The thread initialization failed.", logPrefix);
        m_initializationSuccessful.store(false);
    }

    // set the policy
    if (!setPolicy())
    {
        log()->error("{} The policy and priority cannot be set.", logPrefix);
        m_initializationSuccessful.store(false);
    }

    // Notify that initialization is completed (successful or not)
    {
        // https://en.cppreference.com/w/cpp/thread/condition_variable:
        // Even if the shared variable is atomic, it must be modified while owning the mutex to
        // correctly publish the modification to the waiting thread.
        std::lock_guard<std::mutex> lock(m_cv_mtx);
        m_ready.store(true);
    }
    m_cv.notify_one();

    // cannot continue if the initialization was not successful
    if (!m_initializationSuccessful.load())
    {
        m_state.store(PeriodicThreadState::STOPPED);
        return;
    }

    m_state.store(PeriodicThreadState::INITIALIZED);

    // synchronize the thread
    synchronize();

    // run loop
    m_state.store(PeriodicThreadState::EXECUTING);

    while (m_state == PeriodicThreadState::EXECUTING || m_state == PeriodicThreadState::IDLE)
    {
        this->advance();
    }

    m_state.store(PeriodicThreadState::STOPPED);

    return;
};

bool PeriodicThread::setPolicy()
{
    constexpr auto logPrefix = "[PeriodicThread::setPolicy]";

#ifdef __linux__
    // get the current thread native handle
    pthread_t nativeHandle = pthread_self();
    // get the current thread parameters
    sched_param params;
    params.sched_priority = m_priority;
    // Set the scheduling policy to SCHED_FIFO and priority
    int ret = pthread_setschedparam(nativeHandle, m_policy, &params);
    if (ret != 0)
    {
        log()->error("{} Failed to set scheduling policy, with error: {}", logPrefix, ret);
        if (ret == EPERM)
        {
            log()->error("{} The calling thread does not have the appropriate privileges to set "
                         "the requested scheduling policy and parameters. Try to run the "
                         "YarpLoggerDevice with 'sudo -E'.",
                         logPrefix);
        }
        return false;
    } else
    {
        log()->debug("{} Scheduling policy set to {} with priority {}",
                     logPrefix,
                     m_policy,
                     params.sched_priority);
        return true;
    }
#else
    return true;
#endif
};

#ifdef __linux__
bool PeriodicThread::setPolicy(int policy, int priority)
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
#endif

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

bool PeriodicThread::setMaximumNumberOfAcceptedDeadlineMiss(int maximumNumberOfAcceptedDeadlineMiss)
{
    if (m_state.load() != PeriodicThreadState::INACTIVE)
    {
        BipedalLocomotion::log()->error("[PeriodicThread::setMaximumNumberOfAcceptedDeadlineMiss] "
                                        "The thread has already started. The maximum number of "
                                        "accepted deadline miss cannot be changed.");
        return false;
    }
    m_maximumNumberOfAcceptedDeadlineMiss = maximumNumberOfAcceptedDeadlineMiss;
    return true;
}

int PeriodicThread::getNumberOfDeadlineMiss()
{
    return m_deadlineMiss.load();
}

bool PeriodicThread::enableEarlyWakeUp()
{
    if (m_state.load() != PeriodicThreadState::INACTIVE)
    {
        BipedalLocomotion::log()->error("[PeriodicThread::enableEarlyWakeUp] The thread has "
                                        "already started. The early wake up cannot be changed.");
        return false;
    }
    m_earlyWakeUp = true;
    return true;
}

bool PeriodicThread::threadInit()
{
    return true;
};

bool PeriodicThread::run()
{
    return false;
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
    // stop the thread only if it is executing or idling
    if ((m_state.load() == PeriodicThreadState::EXECUTING
         || m_state.load() == PeriodicThreadState::IDLE))
    {
        m_state.store(PeriodicThreadState::STOPPED);
        BipedalLocomotion::log()->debug("[PeriodicThread::stop] Thread is stopped.");
    }
};

bool PeriodicThread::suspend()
{
    if (m_state.load() == PeriodicThreadState::EXECUTING)
    {
        m_state.store(PeriodicThreadState::IDLE);
        BipedalLocomotion::log()->debug("[PeriodicThread::suspend] Thread is suspended.");
        return true;
    }
    return false;
};

bool PeriodicThread::resume()
{
    if (m_state.load() == PeriodicThreadState::IDLE)
    {
        m_state.store(PeriodicThreadState::EXECUTING);
        BipedalLocomotion::log()->debug("[PeriodicThread::resume] Thread is resumed.");
        return true;
    }
    return false;
};

bool PeriodicThread::start(std::shared_ptr<BipedalLocomotion::System::Barrier> barrier)
{

    std::unique_lock<std::mutex> lock(m_cv_mtx); // lock the mutex for the condition variable

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

    // start the thread
    m_thread = std::thread(threadFunctionLambda);

    // check if the thread is joinable
    if (!m_thread.joinable())
    {
        return false;
    }

    // check if initialization was successful and thread started
    m_cv.wait(lock, [this] { return m_ready.load(); });

    return m_initializationSuccessful.load();
};

bool PeriodicThread::isRunning()
{
    return ((m_state.load() != PeriodicThreadState::INACTIVE)
            && (m_state.load() != PeriodicThreadState::STOPPED));
}

bool PeriodicThread::isInitialized()
{
    return (m_state.load() == PeriodicThreadState::INITIALIZED);
}

void PeriodicThread::advance()
{
    // get the current time
    auto now = BipedalLocomotion::clock().now();

    // busy wait until wake up time
    if (m_earlyWakeUp)
    {
        while (now < m_wakeUpTime)
        {
            now = BipedalLocomotion::clock().now();
        }
    }

    // get the next wake up time
    m_wakeUpTime = now + m_period;

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
    if (BipedalLocomotion::clock().now() > m_wakeUpTime)
    {
        m_deadlineMiss.fetch_add(1); // increment the number of deadline miss
        if (m_maximumNumberOfAcceptedDeadlineMiss > 0)
        {
            if (m_deadlineMiss.load() > m_maximumNumberOfAcceptedDeadlineMiss)
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
    if (m_earlyWakeUp)
    {
        BipedalLocomotion::clock().sleepUntil(m_wakeUpTime - m_schedulerLatency);
    } else
    {
        BipedalLocomotion::clock().sleepUntil(m_wakeUpTime);
    }
};

} // namespace System
} // namespace BipedalLocomotion