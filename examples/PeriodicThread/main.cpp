#include <chrono>
#include <cstdlib>

#include <BipedalLocomotion/System/Barrier.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/PeriodicThread.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

class Thread : public BipedalLocomotion::System::PeriodicThread
{
public:
    Thread();
    ~Thread();
    bool run() override;

    bool threadInit() override;
};

bool Thread::run()
{
    BipedalLocomotion::log()->info("[Thread::run] Thread is running.");
    return true;
}

Thread::Thread()
    : BipedalLocomotion::System::PeriodicThread(std::chrono::milliseconds(1000)){};

Thread::~Thread()
{
    BipedalLocomotion::log()->info("[Thread::~Thread] Thread is destroyed.");
};

bool Thread::threadInit()
{
    BipedalLocomotion::log()->info("[Thread::threadInit] Thread is initialized.");
    return true;
}

int main()
{

    auto barrier = BipedalLocomotion::System::Barrier::create(2);

    // Thread thread;
    auto thread1 = Thread();
    thread1.start(barrier);

    BipedalLocomotion::clock().sleepFor(std::chrono::milliseconds(2000));

    auto thread2 = Thread();
    thread2.setPeriod(std::chrono::seconds(2));
    thread2.start(barrier);

    while (thread1.isRunning())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(4000));
        thread1.stop();
        BipedalLocomotion::log()->info("[main] Thread 1 is asked to stop.");
    }
    BipedalLocomotion::log()->info("[main] About to exit the application.");
    return EXIT_SUCCESS;
}
