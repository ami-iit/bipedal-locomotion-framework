/**
 * @file AdvanceableRunnerTest.cpp
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the GNU Lesser
 * General Public License v2.1 or any later version.
 */

#include <chrono>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/AdvanceableRunner.h>
#include <BipedalLocomotion/System/Barrier.h>
#include <BipedalLocomotion/System/SharedResource.h>
#include <BipedalLocomotion/System/Source.h>
#include <memory>

using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::ParametersHandler;
using namespace std::chrono_literals;

class DummyBlock : public Source<bool>
{
    bool m_output{false};
    std::size_t m_i{0};

public:
    const Output& getOutput() const override
    {
        return m_output;
    }

    bool advance() override
    {
        BipedalLocomotion::log()->info("DummyBlock i = {}", m_i);
        m_i++;
        if (m_i == 10)
        {
            m_output = true;
        }

        return true;
    }

    bool isOutputValid() const override
    {
        return true;
    }
};

TEST_CASE("Test Block")
{
    using namespace std::chrono_literals;
    std::shared_ptr param = std::make_shared<StdImplementation>();

    param->setParameter("sampling_time", 1ms);
    param->setParameter("enable_telemetry", true);
    param->setParameter("name", "Runner");

    std::unique_ptr<DummyBlock> block0 = std::make_unique<DummyBlock>();
    std::unique_ptr<DummyBlock> block1 = std::make_unique<DummyBlock>();

    auto input0 = SharedResource<DummyBlock::Input>::create();
    auto output0 = SharedResource<DummyBlock::Output>::create();

    auto input1 = SharedResource<DummyBlock::Input>::create();
    auto output1 = SharedResource<DummyBlock::Output>::create();

    AdvanceableRunner<DummyBlock> runner0;
    REQUIRE(runner0.initialize(param));
    REQUIRE(runner0.setInputResource(input0));
    REQUIRE(runner0.setOutputResource(output0));
    REQUIRE(runner0.setAdvanceable(std::move(block0)));

    AdvanceableRunner<DummyBlock> runner1;
    REQUIRE(runner1.initialize(param));
    REQUIRE(runner1.setInputResource(input1));
    REQUIRE(runner1.setOutputResource(output1));
    REQUIRE(runner1.setAdvanceable(std::move(block1)));

    SECTION("Without synchronization")
    {
        // run the block
        auto thread0 = runner0.run();
        auto thread1 = runner1.run();

        while (!output0->get() || !output1->get())
        {
            BipedalLocomotion::clock().sleepFor(10ms);
        }

        // close the runner
        runner0.stop();
        runner1.stop();

        // print some information
        BipedalLocomotion::log()->info("Runner0 : Number of deadline miss {}",
                                       runner0.getInfo().deadlineMiss);
        BipedalLocomotion::log()->info("Runner1 : Number of deadline miss {}",
                                       runner1.getInfo().deadlineMiss);

        REQUIRE(output0->get());
        REQUIRE(output1->get());

        // join the treads
        if (thread0.joinable())
        {
            thread0.join();
            thread0 = std::thread();
        }

        if (thread1.joinable())
        {
            thread1.join();
            thread1 = std::thread();
        }
    }

    SECTION("With synchronization")
    {
        constexpr std::size_t numberOfRunners = 2;
        auto barrier = Barrier::create(numberOfRunners);

        // run the block
        auto thread0 = runner0.run(barrier);
        auto thread1 = runner1.run(barrier);

        while (!output0->get() || !output1->get())
        {
            BipedalLocomotion::clock().sleepFor(10ms);
        }

        // close the runner
        runner0.stop();
        runner1.stop();

        // print some information
        BipedalLocomotion::log()->info("Runner0 : Number of deadline miss {}",
                                       runner0.getInfo().deadlineMiss);
        BipedalLocomotion::log()->info("Runner1 : Number of deadline miss {}",
                                       runner1.getInfo().deadlineMiss);

        REQUIRE(output0->get());
        REQUIRE(output1->get());

        // join the treads
        if (thread0.joinable())
        {
            thread0.join();
            thread0 = std::thread();
        }

        if (thread1.joinable())
        {
            thread1.join();
            thread1 = std::thread();
        }
    }
}
