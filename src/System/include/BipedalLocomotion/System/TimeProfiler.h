/**
 * @file TimeProfiler.h
 * @authors Guglielmo Cervettini, Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_TIME_PROFILER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_TIME_PROFILER_H

// std
#include <chrono>
#include <map>
#include <memory>

namespace BipedalLocomotion
{
namespace System
{
/**
 * Timer is a simple class that can be used to measure the time between two events.
 */
class Timer
{
    std::chrono::time_point<std::chrono::steady_clock> m_initTime; /**< Init time. */
    std::chrono::time_point<std::chrono::steady_clock> m_endTime; /**< End time. */
    std::chrono::nanoseconds m_averageDuration; /**< Average duration. */

public:
    /**
     * Reset the average duration.
     */
    void resetAverageDuration();

    /**
     * Set initial time.
     */
    void setInitTime();

    /**
     * Set final time.
     */
    void setEndTime();

    /**
     * Evaluate the average duration.
     */
    void evaluateDuration();

    /**
     * Get the average duration.
     * @return average duration.
     */
    const std::chrono::nanoseconds& getAverageDuration() const;
};

/**
 * TimeProfiler is a simple class that can be used to profile the code.
 */
class TimeProfiler
{
    int m_counter; /**< Counter useful to print the profiling quantities only every m_maxCounter
                      times. */
    int m_maxCounter; /**< The profiling quantities will be printed every maxCounter cycles. */
    std::map<std::string, Timer> m_timers; /**< Dictionary that contains all the timers. */
public:
    /**
     * Set the output period.
     * @param maxCounter is the period (expressed in cycles).
     */
    void setPeriod(int maxCounter);

    /**
     * Add a new timer
     * @param key is the name of the timer.
     * @return true/false in case of success/failure.
     */
    bool addTimer(const std::string& key);

    /**
     * Set the init time for the timer named "key"
     * @param key is the name of the timer.
     * @return true/false in case of success/failure.
     */
    bool setInitTime(const std::string& key);

    /**
     * Set the end time for the timer named "key"
     * @param key is the name of the timer.
     * @return true/false in case of success/failure.
     */
    bool setEndTime(const std::string& key);

    /**
     * Print the profiling quantities.
     */
    void profiling();
};

}; // namespace System
}; // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_TIME_PROFILER_H
