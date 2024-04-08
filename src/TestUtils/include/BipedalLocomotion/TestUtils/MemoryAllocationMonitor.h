/**
 * @file MemoryAllocationMonitor.h
 * @authors Silvio Traversaro
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_MEMORY_ALLOCATION_MONITOR_H
#define BIPEDAL_LOCOMOTION_MEMORY_ALLOCATION_MONITOR_H

//std
#include <cstdint>

namespace BipedalLocomotion {
namespace TestUtils {

class MemoryAllocationMonitor
{
public:
    /**
     * Get if the the memory monitor is actually enabled at the compilation level.
     * The monitor can be enabled (just on Linux) with the FRAMEWORK_RUN_MemoryAllocationMonitor_tests CMake variable
     * @return True if the the memory monitor is enabled, false otherwise.
     */
    static bool monitorIsEnabled();

    /**
     * Start monitoring the memory operations. 
     * This will reset all the memory operaton counters.
     */
    static void startMonitor();

    /**
     * Stop monitoring the memory operations.
     * This will stop any counting of memory operations.
     */
    static void endMonitor();

    /**
     * Check if any memory operation was performed during the last monitor.
     * @return True if no memory operation were performed between the startMonitor and endMonitor calls, false otherwise.
     */
    static bool checkNoMemoryAllocationInLastMonitor();

    /**
     * Equivalent to calling endMonitor() and checkNoMemoryAllocationInLastMonitor() in sequence.
     * @return True if no memory operation were performed between the startMonitor and endMonitor calls, false otherwise.
     */
    static bool endMonitorAndCheckNoMemoryAllocationInLastMonitor();

    /**
     * Return the number of memory operations performed during the last monitor.
     * @return Return the number of memory operations were performed between the startMonitor and endMonitor calls.
     */
    static int32_t getNumberOfDynamicMemoryOperationsInLastMonitor();
};

}
}

#endif // BIPEDAL_LOCOMOTION_GENERIC_CONTAINER_VECTOR_H
