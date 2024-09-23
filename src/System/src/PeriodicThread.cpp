#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/PeriodicThread.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

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

/**
 * @brief PeriodicThread implementation
 */
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
    PeriodicThread* m_owner;
    // thread
    std::thread m_thread;
    // is running
    std::atomic<bool> m_isRunning = false;
    // ask to stop
    std::atomic<bool> m_askToStop = false;
    // force to stop flag
    static std::atomic<bool> _forceToStop;

public:
    /**
     * @brief Constructor
     * @param owner owner of the thread
     * @param period period of the thread
     * @param maximumNumberOfAcceptedDeadlineMiss maximum number of accepted deadline misses
     * @param priority priority of the thread
     * @param policy policy of the thread
     * For example, in Linux,you can set the policy to SCHED_FIFO and priority to 80 in order to
     * improve the real time performances of the thread.
     */
    Impl(PeriodicThread* owner,
         std::chrono::nanoseconds period = std::chrono::nanoseconds(100000),
         int maximumNumberOfAcceptedDeadlineMiss = -1,
         int priority = 0,
         int policy = SCHED_OTHER)
        : m_owner(owner)
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

    /**
     * @brief Destructor, it joins the thread.
     */
    ~Impl()
    {
        if (m_thread.joinable())
        {
            m_thread.join();
        }
    };

    /**
     * @brief Start the thread
     * @return true if the thread was correctly started, false otherwise.
     */
    bool start()
    {
        if (m_isRunning)
        {
            // thread is already running
            BipedalLocomotion::log()->error("[PeriodicThread::start] The thread is already "
                                            "running.");
            return false;
        } else
        {
            m_isRunning = true;
        }

        // lambda wrapper for the thread function
        auto threadFunctionLambda = [this]() { this->threadFunction(); };

        m_thread = std::thread(threadFunctionLambda);

        return m_thread.joinable();
    };

    /**
     * @brief Stop the thread
     */
    void stop()
    {
        m_askToStop = true;
    };

    /**
     * @brief Check if the thread is running
     * @return true if the thread is running, false otherwise.
     */
    bool isRunning()
    {
        return m_isRunning;
    };

private:
    /**
     * @brief Signal handler. It is used to stop the thread when ctrl+c is pressed.
     * @param sig signal
     */
    static void signalHandler(int sig)
    {
        if (sig == SIGINT)
        {
            _forceToStop = true;
        }
    }

    /**
     * @brief Step function. It is called at each iteration of the thread. Advances the thread by
     * one step. It runs the user defined function and oversees the thread timing and resources.
     */
    void step()
    {
        // get the current time
        auto now = BipedalLocomotion::clock().now();
        // get the next wake up time
        auto nextWakeUpTime = now + m_period;

        // run user defined function
        if (!m_owner->run())
        {
            m_isRunning = false;
            return;
        }

        // check if the thread has to stop
        if (m_askToStop)
        {
            m_isRunning = false;
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

    /**
     * @brief Run function. It is the main function of the thread.
     */
    void run()
    {
        while ((m_isRunning) || !(_forceToStop))
        {
            step();
        }
    };

    /**
     * @brief Set the policy of the thread.
     * @return true if the policy was correctly set, false otherwise.
     */
    bool setPolicy()
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

    /**
     * @brief Initialize the thread.
     * @return true if the thread was correctly initialized, false otherwise.
     */
    bool threadInit()
    {
        return m_owner->threadInit();
    }

    /**
     * @brief Thread function. It is the function passed to std::thread().
     */
    void threadFunction()
    {
        constexpr auto logPrefix = "[PeriodicThread::threadFunction]";

        if (!threadInit())
        {
            BipedalLocomotion::log()->error("{} Failed to initialize the thread", logPrefix);
            return;
        }
        if (!setPolicy())
        {
            BipedalLocomotion::log()->error("{} Failed to set the policy", logPrefix);
            return;
        }
        run();
    };

}; // class Impl

// static member initialization
std::atomic<bool> PeriodicThread::Impl::_forceToStop = false;

PeriodicThread::PeriodicThread(std::chrono::nanoseconds period,
                               int maximumNumberOfAcceptedDeadlineMiss,
                               int priority,
                               int policy)
{
    m_impl = std::make_unique<Impl>(this,
                                    period,
                                    maximumNumberOfAcceptedDeadlineMiss,
                                    priority,
                                    policy);
};

PeriodicThread::~PeriodicThread(){};

bool PeriodicThread::threadInit()
{
    return true;
};

void PeriodicThread::stop()
{
    m_impl->stop();
};

bool PeriodicThread::start()
{
    return m_impl->start();
}

bool PeriodicThread::isRunning()
{
    return m_impl->isRunning();
}

} // namespace System
} // namespace BipedalLocomotion