/**
 * @file AdvanceableRunner.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_ADVANCEABLE_RUNNER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_ADVANCEABLE_RUNNER_H

#include <BipedalLocomotion/GenericContainer/TemplateHelpers.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/SharedResource.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <atomic>
#include <memory>
#include <thread>
#include <type_traits>

namespace BipedalLocomotion
{
namespace System
{

/**
 * AdvanceableRunner is an helper class that can be used to run a advanceable at a given period.
 * Different AdvanceableRunners can communicate trough the SharedResource class. The
 * AdvanceableRunner can be used only with Advanceable. Do not try to use if with a class that does
 * not inherit from advanceable, a compile time assert will be thrown. This is a visual example of
 * usage <br/> <img
 * src="https://user-images.githubusercontent.com/16744101/112768882-7c137280-901e-11eb-8c24-05b7cc3fa5e5.png"
 * alt="AdvanceableRunner" width="500"> Here we want to run two advanceables in parallel with a
 * different sampling time. The AdvanceableRunner is a class that helps you in creating a periodic
 * thread. The SharedResource is the way that can be used to communicate between two advanceables
 * running in separate threads (i.e. the wired between two AdvanceableRunners)
 */
template <class _Advanceable> class AdvanceableRunner
{
    static_assert(BipedalLocomotion::is_base_of_template<System::Advanceable, _Advanceable>::value,
                  "The _Advanceable class must be derived from System::Advanceable class.");

public:
    using Input = typename _Advanceable::Input;
    using Output = typename _Advanceable::Output;

private:
    bool m_isInitialized{false}; /**< True if the runner is initialized */
    std::chrono::duration<double> m_dT{std::chrono::duration_values<double>::zero()}; /**< Period of
                                                                                         the runner
                                                                                       */
    std::atomic<bool> m_isRunning{false}; /**> If True the runner is running */

    std::unique_ptr<_Advanceable> m_advanceable; /**< Advanceable contained in the runner */
    typename SharedResource<Input>::Ptr m_input; /**< Input shared resource */
    typename SharedResource<Output>::Ptr m_output; /**< Output shared resource */

    bool m_isTelemetryEnabled{false}; /**< If true some additional information will be stored */
    struct Info
    {
        unsigned int deadlineMiss{0}; /**< Number of deadline miss */
        double dT{-1.0}; /**< Period of the runner */
    };
    std::mutex m_infoMutex; /**< Mutex used to protect the information struct */
    Info m_info; /**< Information struct */

public:
    /**
     * Initialize the AdvanceableRunner class
     * @param handler pointer to a parameter handler
     * @note The following parameters are required
     * |   Parameter Name   |   Type   |                              Description | Mandatory |
     * |:------------------:|:--------:|:---------------------------------------------------------------------:|:---------:|
     * |   `sampling_time`  | `double` |   Strictly positive number rapresenting the sampling time
     * in seconds  |    Yes    | | `enable_telemetry` |  `bool`  | If True some additional
     * information are stored. Default value `false` |     No    |
     * @return true in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);

    /**
     * Set the advanceable inside the runner.
     * @param advanceable an unique pointer representing the advanceable
     * @return true in case of success, false otherwise
     */
    bool setAdvanceable(std::unique_ptr<_Advanceable> advanceable);

    /**
     * Set the input resource
     * @param resource pointer representing the input resource
     * @return true in case of success, false otherwise
     */
    bool setInputResource(std::shared_ptr<SharedResource<Input>> resource);

    /**
     * Set the output resource
     * @param resource pointer representing the output resource
     * @return true in case of success, false otherwise
     */
    bool setOutputResource(std::shared_ptr<SharedResource<Output>> resource);

    /**
     * Get some info of the runner,
     * @return A copy of the Info struct
     */
    Info getInfo();

    /**
     * Run the advanceable runner. The function create a periodic thread running with a period of
     * m_dT seconds.
     * @return a thread of containing the running process. If the runner was not correctly
     * initialized the thread is invalid.
     */
    std::thread run();

    /**
     * Stop the thread
     * @return true in case of success, false otherwise
     */
    void stop();
};

template <class _Advanceable>
bool AdvanceableRunner<_Advanceable>::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[AdvanceableRunner::initialize]";

    if (m_isInitialized)
    {
        log()->error("{} AdvanceableRunner is already initialized.", errorPrefix);
        return false;
    }

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameters handler is not valid.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("sampling_time", m_info.dT))
    {
        log()->error("{} Unable to get the sampling time.", errorPrefix);
        return false;
    }

    if (m_info.dT <= 0)
    {
        log()->error("{} The sampling time must be a strictly positive number.", errorPrefix);
        return false;
    }

    m_dT = std::chrono::duration<double>(m_info.dT);

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

template <class _Advanceable>
bool AdvanceableRunner<_Advanceable>::setAdvanceable(std::unique_ptr<_Advanceable> advanceable)
{
    constexpr auto errorPrefix = "[AdvanceableRunner::setAdvanceable]";

    if (advanceable == nullptr)
    {
        log()->error("{} The advanceable is not valid.", errorPrefix);
        return false;
    }

    m_advanceable = std::move(advanceable);
    return true;
}

template <class _Advanceable>
bool AdvanceableRunner<_Advanceable>::setInputResource(std::shared_ptr<SharedResource<Input>> input)
{
    constexpr auto logPrefix = "[AdvanceableRunner::setInputResource]";

    if (input == nullptr)
    {
        log()->error("{} The input is not valid.", logPrefix);
        return false;
    }

    this->m_input = input;

    return true;
}

template <class _Advanceable>
bool AdvanceableRunner<_Advanceable>::setOutputResource(
    std::shared_ptr<SharedResource<Output>> output)
{
    constexpr auto logPrefix = "[AdvanceableRunner::setOutputResource]";

    if (output == nullptr)
    {
        log()->error("{} The input is not valid.", logPrefix);
        return false;
    }

    this->m_output = output;

    return true;
}

template <class _Advanceable> std::thread AdvanceableRunner<_Advanceable>::run()
{
    constexpr auto logPrefix = "[AdvanceableRunner::run]";

    if (!m_isInitialized)
    {
        log()->warn("{} The AdvanceableRunner class is not initialized. An invalid thread will be "
                    "returned",
                    logPrefix);
        assert(false && "[AdvanceableRunner::run] The thread is already running.");
        return std::thread();
    }

    if (m_input == nullptr || m_output == nullptr)
    {
        log()->warn("{} The shared resources are not valid. An invalid thread will be returned.",
                    logPrefix);
        assert(false && "[AdvanceableRunner::run] The shared resources are not valid.");
        return std::thread();
    }

    if (m_advanceable == nullptr)
    {
        log()->warn("{} The advanceable is not valid. An invalid thread will be returned.",
                    logPrefix);
        assert(false && "[AdvanceableRunner::run] The advanceable is not valid.");
        return std::thread();
    }

    if (m_isRunning)
    {
        log()->warn("{} The thread is already running. An invalid thread will be returned.",
                    logPrefix);
        assert(false && "[AdvanceableRunner::run] The thread is already running.");
        return std::thread();
    }

    // run the thread
    m_isRunning = true;
    auto function = [&]() -> bool {
        while (this->m_isRunning)
        {
            const auto wakeUpTime = BipedalLocomotion::clock().now() + m_dT;
            if (!this->m_advanceable->setInput(this->m_input->get()))
            {
                m_isRunning = false;
                log()->error("{} Unable to set the input to the advanceable.", logPrefix);
                break;
            }

            // advance the advanceable
            if (!this->m_advanceable->advance())
            {
                m_isRunning = false;
                log()->error("{} Unable to advance the advanceable.", logPrefix);
                break;
            }
            assert(this->m_advanceable->isOutputValid());

            // set the output
            this->m_output->set(this->m_advanceable->getOutput());

            // release the CPU
            BipedalLocomotion::clock().yield();

            // this is enabled only if telemetry is set to true.
            if (m_isTelemetryEnabled && wakeUpTime < BipedalLocomotion::clock().now())
            {
                std::lock_guard<std::mutex> guard(m_infoMutex);
                m_info.deadlineMiss++;
            }

            BipedalLocomotion::clock().sleepUntil(wakeUpTime);
        }

        return this->m_advanceable->close();
    };

    return std::thread(function);
}

template <class _Advanceable> void AdvanceableRunner<_Advanceable>::stop()
{
    // m_isRunning is an atomic<bool> the mutex is not required here
    m_isRunning = false;
}

template <class _Advanceable>
typename AdvanceableRunner<_Advanceable>::Info AdvanceableRunner<_Advanceable>::getInfo()
{
    std::lock_guard<std::mutex> guard(m_infoMutex);
    return m_info;
}

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_ADVANCEABLE_RUNNER_H
