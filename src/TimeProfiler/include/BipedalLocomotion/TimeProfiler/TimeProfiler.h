/**
 * @file TimeProfiler.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef BIPEDAL_LOCOMOTION_TIME_PROFILER_H
#define BIPEDAL_LOCOMOTION_TIME_PROFILER_H

// std
#include <map>
#include <memory>

namespace BipedalLocomotion
{

    /**
     * Simple timer.
     */
    class Timer
    {
        clock_t m_initTime; /**< Init time. */
        clock_t m_endTime; /**< End time. */
        double m_averageDuration; /**< Average duration. */

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
         * Evalyate the average duration.
         */
        void evaluateDuration();

        /**
         * Get the average duration.
         * @return average duration.
         */
        const double& getAverageDuration() const;

    };

    /**
     * Simple Time profiler class
     */
    class TimeProfiler
    {
        int m_counter; /**< Counter useful to print the profiling quantities only every m_maxCounter times. */
        int m_maxCounter; /**< The profiling quantities will be printed every maxCounter cycles. */
        std::map<std::string, std::unique_ptr<Timer>> m_timers; /**< Dictionary that contains all the timers. */

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
};

#endif
