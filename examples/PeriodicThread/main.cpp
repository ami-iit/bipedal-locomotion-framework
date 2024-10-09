#include <chrono>
#include <cstdlib>
#include <string>

#include <BipedalLocomotion/System/Barrier.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/PeriodicThread.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

class Thread : public BipedalLocomotion::System::PeriodicThread
{
public:
    std::string name = "Thread";
    Thread(std::string name);
    ~Thread();
    bool run() override;

    bool threadInit() override;
};

bool Thread::run()
{
    BipedalLocomotion::log()->info("[Thread::run] {} is running.", name);
    return true;
}

Thread::Thread(std::string name)
    : name(name)
    , BipedalLocomotion::System::PeriodicThread(std::chrono::milliseconds(1000)){};

Thread::~Thread()
{
    BipedalLocomotion::log()->info("[Thread::~Thread] {} is destroyed.", name);
};

bool Thread::threadInit()
{
    BipedalLocomotion::log()->info("[Thread::threadInit] {} is initialized.", name);
    return true;
}

int main()
{

    auto barrier = BipedalLocomotion::System::Barrier::create(2);

    // Thread thread;
    auto thread1 = Thread("Thread 1");
    thread1.start(barrier);

    BipedalLocomotion::clock().sleepFor(std::chrono::milliseconds(2000));

    auto thread2 = Thread("Thread 2");
    thread2.setPeriod(std::chrono::seconds(2));
    thread2.start(barrier);

    while (thread1.isRunning() || thread2.isRunning())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(4000));
        thread1.stop();
        BipedalLocomotion::log()->info("[main] Thread 1 is asked to stop.");

        if (!thread1.isRunning())
        {
            thread2.stop();
            BipedalLocomotion::log()->info("[main] Thread 2 is asked to stop.");
        }
    }
    BipedalLocomotion::log()->info("[main] About to exit the application.");
    return EXIT_SUCCESS;
}
