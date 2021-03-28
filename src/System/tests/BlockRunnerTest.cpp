/**
 * @file BlockRunnerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include <BipedalLocomotion/System/Block.h>
#include <BipedalLocomotion/System/BlockRunner.h>
#include <BipedalLocomotion/System/SharedResource.h>
#include <BipedalLocomotion/System/impl/traits.h>

namespace BipedalLocomotion
{
namespace System
{
class DummyBlock;
}
} // namespace BipedalLocomotion

BLF_DEFINE_SYSTEM_BLOCK_INTERAL_STRUCTURE(DummyBlock, BipedalLocomotion::System::EmptyType, bool);

namespace BipedalLocomotion
{
namespace System
{
class DummyBlock : public Block<DummyBlock>
{
    bool m_output{false};
    std::size_t m_i{0};

public:
    const Output& getOutput() const
    {
        return m_output;
    }

    bool setInput(const Input& input)
    {
        return true;
    }

    bool advance()
    {
        log()->info("DummyBlock i = {}", m_i);

        m_i++;
        if (m_i == 10)
        {
            m_output = true;
        }

        return true;
    }

    bool close()
    {
        return true;
    }
};
} // namespace System
} // namespace BipedalLocomotion

TEST_CASE("Test Block")
{
    using namespace BipedalLocomotion::System;
    using namespace BipedalLocomotion::ParametersHandler;
    std::shared_ptr param = std::make_shared<StdImplementation>();

    param->setParameter("sampling_time", 0.05);
    param->setParameter("enable_telemetry", true);

    std::unique_ptr<DummyBlock> block0 = std::make_unique<DummyBlock>();
    std::unique_ptr<DummyBlock> block1 = std::make_unique<DummyBlock>();

    auto input0 = SharedResource<DummyBlock::Input>::Create();
    auto output0 = SharedResource<DummyBlock::Output>::Create();

    auto input1 = SharedResource<DummyBlock::Input>::Create();
    auto output1 = SharedResource<DummyBlock::Output>::Create();


    BlockRunner<DummyBlock> runner0;
    REQUIRE(runner0.initialize(param));
    REQUIRE(runner0.setInputResource(input0));
    REQUIRE(runner0.setOutputResource(output0));
    REQUIRE(runner0.setBlock(std::move(block0)));


    BlockRunner<DummyBlock> runner1;
    REQUIRE(runner1.initialize(param));
    REQUIRE(runner1.setInputResource(input1));
    REQUIRE(runner1.setOutputResource(output1));
    REQUIRE(runner1.setBlock(std::move(block1)));


    // run the block
    auto thread0 = runner0.run();
    auto thread1 = runner1.run();

    while (!output0->get() && !output1->get())
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
    }

    // close the runner
    runner0.close();
    runner1.close();

    BipedalLocomotion::log()->info("Runner0 : Number of deadline miss {}",
                                   runner0.getInfo().deadlineMiss);
    BipedalLocomotion::log()->info("Runner1 : Number of deadline miss {}",
                                   runner1.getInfo().deadlineMiss);

    // joint the tread
    thread0.join();
    thread1.join();

}
