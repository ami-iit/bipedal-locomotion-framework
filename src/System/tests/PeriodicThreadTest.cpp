/**
 * @file PeriodicThreadTest.cpp
 * @authors Lorenzo Moretti
 * @copyright This software may be modified and distributed under the terms of the GNU Lesser
 * General Public License v2.1 or any later version.
 */

#include <atomic>
#include <chrono>
#include <sched.h>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/System/Barrier.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/PeriodicThread.h>

using namespace BipedalLocomotion::System;
using namespace std::chrono_literals;

class Thread : public PeriodicThread
{
protected:
    bool threadInit() override
    {
        m_counter.store(0);
        return true;
    }
    bool run() override
    {
        m_counter++;
        return true;
    }

private:
    std::atomic<int> m_counter{0};

public:
    int getCounter()
    {
        return m_counter.load();
    }
};

class ThreadWithDeadlineMiss : public PeriodicThread
{
protected:
    bool threadInit() override
    {
        return true;
    }
    bool run() override
    {
        BipedalLocomotion::clock().sleepFor(waitTime);
        return true;
    }

private:
    std::chrono::nanoseconds waitTime;

public:
    void setWaitTime(std::chrono::nanoseconds time)
    {
        waitTime = time;
    }
};

TEST_CASE("Test Periodic Thread", "[PeriodicThread]")
{
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::StdClockFactory>());

    auto period = 50ms;

    // create
    auto thread = Thread();

    // set the period
    REQUIRE(thread.setPeriod(period));

    // set the policy
    REQUIRE(thread.setPolicy(SCHED_OTHER, 0));

    // start the thread
    REQUIRE(thread.start());

    BipedalLocomotion::clock().sleepFor(period);

    // check if the thread is running
    REQUIRE(thread.isRunning());

    // suspend the thread
    REQUIRE(thread.suspend());
    BipedalLocomotion::clock().sleepFor(period);

    // check if the thread is suspended
    int counter = thread.getCounter();
    BipedalLocomotion::clock().sleepFor(period);
    REQUIRE(counter == thread.getCounter());

    // resume the thread
    REQUIRE(thread.resume());
    BipedalLocomotion::clock().sleepFor(period);

    // stop the thread
    thread.stop();
    BipedalLocomotion::clock().sleepFor(period);

    // check if the thread is stopped
    REQUIRE(!thread.isRunning());
}

TEST_CASE("Test Periodic Thread", "[PeriodicThreadSynchronization]")
{
    auto period = 100ms;

    auto barrier = BipedalLocomotion::System::Barrier::create(2);

    // create two threads
    auto thread1 = Thread();
    thread1.setPeriod(period);

    auto thread2 = Thread();
    thread2.setPeriod(period);

    // start thread 1
    REQUIRE(thread1.start(barrier));
    BipedalLocomotion::clock().sleepFor(2 * period);

    // check that the thread 1 is waiting for thread 2
    // (i.e. threadInit has been called, but run has not been called yet)
    REQUIRE(thread1.isInitialized());
    REQUIRE(!thread2.isRunning());

    // start thread 2
    REQUIRE(thread2.start(barrier));
    BipedalLocomotion::clock().sleepFor(period);

    // check that the thread 2 is running
    REQUIRE(thread2.isRunning());

    // stop the threads
    thread1.stop();
    thread2.stop();
}

TEST_CASE("Test Periodic Thread", "[PeriodicThreadNotAllowed]")
{

    auto period = 50ms;

    // create
    auto thread = Thread();

    // start the thread
    REQUIRE(thread.start());
    BipedalLocomotion::clock().sleepFor(period);

    // try to set the period
    REQUIRE(!thread.setPeriod(period));

    // try to set the policy
    REQUIRE(!thread.setPolicy(SCHED_OTHER, 0));

    // try to set the maximum number of accepted deadline miss
    REQUIRE(!thread.setMaximumNumberOfAcceptedDeadlineMiss(0));

    // try to enable early wake up
    REQUIRE(!thread.enableEarlyWakeUp());

    // stop the thread
    thread.stop();
    BipedalLocomotion::clock().sleepFor(period);

    // try to resume the thread
    REQUIRE(!thread.resume());
}

TEST_CASE("Test Periodic Thread", "[PeriodicThreadDeadlineMiss]")
{
    auto period = 50ms;

    // create
    auto thread = ThreadWithDeadlineMiss();

    // set the period
    REQUIRE(thread.setPeriod(period));

    // set the wait time in the run function to trigger a deadline miss
    thread.setWaitTime(2 * period);

    // set the maximum number of accepted deadline miss
    REQUIRE(thread.setMaximumNumberOfAcceptedDeadlineMiss(2));

    // start the thread
    REQUIRE(thread.start());

    BipedalLocomotion::clock().sleepFor(10 * period);

    // check if the thread is stopped
    REQUIRE(!thread.isRunning());

    // check the number of deadline miss
    REQUIRE(thread.getNumberOfDeadlineMiss() == 3);
}