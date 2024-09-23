#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/PeriodicThread.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <ctime>
#include <memory>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <thread>

namespace BipedalLocomotion
{
namespace System
{

class PeriodicThread::Impl
{
    // maximum number of accepted deadline misses
    int m_maximumNumberOfAcceptedDeadlineMiss;
    // deadline miss
    int m_deadlineMiss;
    // scheuling policy
    int m_policy;
    // priority
    int m_priority;
    // period
    std::chrono::nanoseconds m_period;
    // owner
    std::unique_ptr<PeriodicThread> m_owner;
    // thread
    std::thread m_thread;
    // is running
    bool m_isRunning = false;
    // stop flag
    static std::atomic<bool> _askToStop;

public:
    Impl(std::unique_ptr<PeriodicThread> owner,
         std::chrono::nanoseconds period = std::chrono::nanoseconds(100000),
         int maximumNumberOfAcceptedDeadlineMiss = -1,
         int priority = 0,
         int policy = SCHED_OTHER)
        : m_owner(std::move(owner))
        , m_period(period)
        , m_thread()
        , m_deadlineMiss(0)
        , m_maximumNumberOfAcceptedDeadlineMiss(maximumNumberOfAcceptedDeadlineMiss)
        , m_priority(priority)
        , m_policy(policy)
        , m_isRunning(false)
    {
        // set the signal handler for ctrl+c
        std::signal(SIGINT, signalHandler);
    };

private:
    static void signalHandler(int sig)
    {
        _askToStop = true;
    }

    void step()
    {
        // get the current time
        auto now = BipedalLocomotion::clock().now();
        // get the next wake up time
        auto nextWakeUpTime = now + m_period;

        // run user defined function
        m_owner->run();

        // check if the deadline is missed
        if (nextWakeUpTime < BipedalLocomotion::clock().now())
        {
            m_deadlineMiss++;
            if (m_maximumNumberOfAcceptedDeadlineMiss > 0)
            {
                if (m_deadlineMiss > m_maximumNumberOfAcceptedDeadlineMiss)
                {
                    // we have to close the runner
                    m_isRunning = false;
                    return;
                }
            }
        }

        // yield the CPU
        BipedalLocomotion::clock().yield();

        // wait until the next deadline
        BipedalLocomotion::clock().sleepUntil(nextWakeUpTime);
    };

    void run()
    {
        while ((m_isRunning) || !(_askToStop))
        {
            step();
        }
        stop();
    };

    void start()
    {
        m_isRunning = true;

        // lambda wrapper for the thread function
        auto threadFunctionLambda = [this]() { this->threadFunction(); };

        m_thread = std::thread(threadFunctionLambda);
    };

    void setPolicy()
    {
        // get the current thread native handle
        pthread_t nativeHandle = pthread_self();
        // get the current thread parameters
        sched_param params;
        params.sched_priority = m_priority;
        // set the new policy
        pthread_setschedparam(nativeHandle, m_policy, &params);
    };

    void threadInit()
    {
        m_owner->threadInit();
    }

    void threadFunction()
    {
        threadInit();
        setPolicy();
        run();
    };

    void stop()
    {
        m_owner->stop();
    };

    // destructor
    ~Impl()
    {
        if (m_thread.joinable())
        {
            m_thread.join();
        }
    };
};

// static member initialization
std::atomic<bool> PeriodicThread::Impl::_askToStop = false;

PeriodicThread::PeriodicThread(std::chrono::nanoseconds period,
                               int maximumNumberOfAcceptedDeadlineMiss,
                               int priority,
                               int policy)
{
    m_impl = std::make_unique<Impl>(std::unique_ptr<PeriodicThread>(this),
                                    period,
                                    maximumNumberOfAcceptedDeadlineMiss,
                                    priority,
                                    policy);
}

} // namespace System
} // namespace BipedalLocomotion

// main function
int main()
{

    return 0;
}