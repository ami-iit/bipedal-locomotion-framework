/**
 * @file BlockRunner.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_BLOCK_RUNNER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_BLOCK_RUNNER_H

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Block.h>
#include <BipedalLocomotion/System/SharedResource.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <type_traits>

namespace BipedalLocomotion
{
namespace System
{

/**
 * BlockRunner is an helper class that can be used to run a block at a given period. Different
 * BlockRunners can communicate trough the SharedResource class. The BlockRunner can be used only
 * with Block. Do not try to use if with a class that does not inherit from block, a compile time
 * assert will be thrown.
 * This is a visual example of usage
 * <br/>
 * <img
 * src="https://user-images.githubusercontent.com/16744101/112768882-7c137280-901e-11eb-8c24-05b7cc3fa5e5.png" alt="BlockRunner" width="500">
 * Here we want to run two blocks in parallel with a different sampling time. The BlockRunner is
 *  a class that helps you in creating a periodic thread. The SharedResource is the way that can
 *  be used to communicate between two blocks running in separate threads (i.e. the wired between
 * two BlockRunners)
 */
template <class _Block> class BlockRunner
{
    static_assert(std::is_base_of<BipedalLocomotion::System::Block<_Block>, _Block>::value,
                  "The _Block class must be derived from System::Block class.");

public:
    using BlockInput = typename _Block::Input;
    using BlockOutput = typename _Block::Output;

private:
    bool m_isInitialized{false}; /**< True if the runner is initialized */
    double m_dT{-1.0};     /**< Period of the runner */
    std::atomic<bool> m_isRunning{false}; /**> If True the runner is running */

    std::unique_ptr<_Block> m_block; /**< Block contained in the runner */
    typename SharedResource<BlockInput>::Ptr m_input; /**< Input shared resource */
    typename SharedResource<BlockOutput>::Ptr m_output; /**< Output shared resource */

    bool m_isTelemetryEnabled{false}; /**< If true some additional information will be stored */
    struct Info
    {
        unsigned int deadlineMiss{0}; /**< Number of deadline miss */
        double dT{-1.0}; /**< Period of the runner */
    };
    std::mutex m_infoMutex; /**< Mutex used to protect the information struct */
    Info m_info; /**< Information struct */

    /**
     * Function containing the default clock used to run the runner
     */
    static double defaultClock()
    {
        return std::chrono::time_point_cast<std::chrono::duration<double>>(
                   std::chrono::high_resolution_clock::now())
            .time_since_epoch()
            .count();
    }
    std::function<double(void)> now{defaultClock};

public:
    /**
     * Initialize the BlockRunner class
     * @param handler pointer to a parameter handler
     * @note The following parameters are required
     * |   Parameter Name   |   Type   |                              Description                              | Mandatory |
     * |:------------------:|:--------:|:---------------------------------------------------------------------:|:---------:|
     * |   `sampling_time`  | `double` |   Strictly positive number rapresenting the sampling time in seconds  |    Yes    |
     * | `enable_telemetry` |  `bool`  | If True some additional information are stored. Default value `false` |     No    |
     * @return true in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler);

    /**
     * Set the block inside the runner.
     * @param block an unique pointer representing the block
     * @return true in case of success, false otherwise
     */
    bool setBlock(std::unique_ptr<_Block> block);

    /**
     * Set the input resource
     * @param resource pointer representing the input resource
     * @return true in case of success, false otherwise
     */
    bool setInputResource(std::shared_ptr<SharedResource<BlockInput>> resource);

    /**
     * Set the output resource
     * @param resource pointer representing the output resource
     * @return true in case of success, false otherwise
     */
    bool setOutputResource(std::shared_ptr<SharedResource<BlockOutput>> resource);

    /**
     * Get some info of the runner,
     * @return A copy of the Info struct
     */
    Info getInfo();

    /**
     * Run the block runner. The function create a periodic thread running with a period of m_dT
     * seconds.
     * @return a thread of containing the running process. If the runner was not correctly
     * initialized the thread is invalid.
     */
    std::thread run();

    /**
     * Stop the tread and close the block.
     * @return true in case of success, false otherwise
     */
    bool close();
};

template <class _Block>
bool BlockRunner<_Block>::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[BlockRunner::initialize]";

    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} The parameters handler is not valid.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("sampling_time", m_dT))
    {
        log()->error("{} Unable to get the sampling time.", errorPrefix);
        return false;
    }

    if (m_dT <= 0)
    {
        log()->error("{} The sampling time must be a strictly positive number.", errorPrefix);
        return false;
    }

    m_info.dT = m_dT;

    if (!ptr->getParameter("enable_telemetry", m_isTelemetryEnabled))
    {
        log()->info("{} enable_telemetry parameter not found. The default parameter will be used: "
                    "{}.",
                    errorPrefix,
                    m_isTelemetryEnabled);
    }

    m_isInitialized = true;
    return true;
}

template <class _Block> bool BlockRunner<_Block>::setBlock(std::unique_ptr<_Block> block)
{
    constexpr auto errorPrefix = "[BlockRunner::setBlock]";

    if (block == nullptr)
    {
        log()->error("{} The block is not valid.", errorPrefix);
        return false;
    }

    m_block = std::move(block);
    return true;
}

template <class _Block>
bool BlockRunner<_Block>::setInputResource(std::shared_ptr<SharedResource<BlockInput>> input)
{
    constexpr auto logPrefix = "[BlockRunner::setInputResource]";

    if (input == nullptr)
    {
        log()->error("{} The input is not valid.", logPrefix);
        return false;
    }

    this->m_input = input;

    return true;
}

template <class _Block>
bool BlockRunner<_Block>::setOutputResource(std::shared_ptr<SharedResource<BlockOutput>> output)
{
    constexpr auto logPrefix = "[BlockRunner::setOutputResource]";

    if (output == nullptr)
    {
        log()->error("{} The input is not valid.", logPrefix);
        return false;
    }

    this->m_output = output;

    return true;
}

template <class _Block> std::thread BlockRunner<_Block>::run()
{
    constexpr auto logPrefix = "[BlockRunner::run]";

    if (!m_isInitialized)
    {
        log()->warn("{} The BlockRunner class is not initialized. An invalid thread will be "
                    "returned",
                    logPrefix);
        assert(false && "[BlockRunner::run] The thread is already running.");
        return std::thread();
    }

    if (m_input == nullptr || m_output == nullptr)
    {
        log()->warn("{} The shared resources are not valid. An invalid thread will be "
                    "returned.",
                    logPrefix);
        assert(false && "[BlockRunner::run] The shared resources are not valid.");
        return std::thread();
    }

    if (m_block == nullptr)
    {
        log()->warn("{} The block is not valid. An invalid thread will be returned.", logPrefix);
        assert(false && "[BlockRunner::run] The block is not valid.");
        return std::thread();
    }

    if (m_isRunning)
    {
        log()->warn("{} The thread is already running. An invalid thread will be returned.",
                    logPrefix);
        assert(false && "[BlockRunner::run] The thread is already running.");
        return std::thread();
    }

    // run the thread
    m_isRunning = true;
    auto function = [&]() -> bool {
        while (this->m_isRunning)
        {
            auto start = this->now();
            if (!this->m_block->setInput(this->m_input->get()))
            {
                m_isRunning = false;
                log()->error("{} Unable to set the input to the block.", logPrefix);
                return false;
            }

            if (!this->m_block->advance())
            {
                m_isRunning = false;
                log()->error("{} Unable to advance the block.", logPrefix);
                return false;
            }

            this->m_output->set(this->m_block->getOutput());

            const auto elapsed = this->now() - start;
            const auto timeToWait = m_dT - elapsed;

            // this is enabled only if telemetry is set to true.
            if (m_isTelemetryEnabled)
            {
                std::lock_guard<std::mutex> guard(m_infoMutex);

                if (timeToWait <= 0)
                {
                    m_info.deadlineMiss++;
                }
            }

            std::this_thread::sleep_for(std::chrono::duration<double>(timeToWait));
        }
        return  true;
    };

    return std::thread(function);
}

template <class _Block> bool BlockRunner<_Block>::close()
{
    // m_isRunning is an atomic<bool> the mutex is not required here
    m_isRunning = false;
    return m_block->close();
}

template <class _Block> typename BlockRunner<_Block>::Info BlockRunner<_Block>::getInfo()
{
    std::lock_guard<std::mutex> guard(m_infoMutex);
    return m_info;
}

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_BLOCK_RUNNER_H
