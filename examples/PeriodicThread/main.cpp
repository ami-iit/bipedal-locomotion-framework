#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/PeriodicThread.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <chrono>
#include <iostream>

class Thread : public BipedalLocomotion::System::PeriodicThread
{
public:
    Thread();
    bool run() override;

    bool threadInit() override;
};

bool Thread::run()
{

    BipedalLocomotion::clock().sleepFor(std::chrono::milliseconds(500));
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
    std::cerr << "Thread class pointer id: " << &thread << std::endl;
    thread.start();
    return EXIT_SUCCESS;
}
