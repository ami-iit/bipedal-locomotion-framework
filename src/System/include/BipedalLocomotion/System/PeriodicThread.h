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

    // Virtual destructor
    virtual ~PeriodicThread() = default;

    /**
     * @brief This method is called at each iteration of the thread
     */
    virtual void run() = 0;
    /**
     * @brief This method is called at the beginning of the thread
     */
    virtual void threadInit() = 0;
    /**
     * @brief This method is called at the end of the thread
     */
    virtual void stop() = 0;

private:
    // private implementation
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
} // namespace System
} // namespace BipedalLocomotion