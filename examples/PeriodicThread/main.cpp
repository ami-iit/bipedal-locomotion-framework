#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/PeriodicThread.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <chrono>

class Thread : public BipedalLocomotion::System::PeriodicThread
{
public:
    Thread();
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

bool Thread::threadInit()
{
    BipedalLocomotion::log()->info("[Thread::threadInit] Thread is initialized.");
    return true;
}

int main()
{
    // Thread thread;
    auto thread = Thread();
    thread.start();

    while (thread.isRunning())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        thread.stop();
        BipedalLocomotion::log()->info("[main] Thread is asked to stop.");
    }
    return EXIT_SUCCESS;
}
