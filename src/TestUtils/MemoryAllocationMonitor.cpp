/**
 * @file MemoryAllocationMonitor.cpp
 * @authors Silvio Traversaro
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TestUtils/MemoryAllocationMonitor.h>

#include "MemoryAllocationMonitorGlobalVariables.h"

using namespace BipedalLocomotion::TestUtils;

// Definition of global declared in MemoryAllocationMonitorGlobalVariables.h 
int32_t g_blf_mam_monitorEnabled = 0;
int32_t g_blf_mam_numberOfCallocOperations = 0;
int32_t g_blf_mam_numberOfMallocOperations = 0;
int32_t g_blf_mam_numberOfFreeOperations = 0;
int32_t g_blf_mam_numberOfReallocOperations = 0;
int32_t g_blf_mam_numberOfMemalignOperations = 0;

bool MemoryAllocationMonitor::monitorIsEnabled()
{
    return true;
}

void MemoryAllocationMonitor::startMonitor()
{
    // Enable monitor
    g_blf_mam_monitorEnabled = 1;

    // Reset counters
    g_blf_mam_numberOfCallocOperations = 0;
    g_blf_mam_numberOfMallocOperations = 0;
    g_blf_mam_numberOfFreeOperations = 0;
    g_blf_mam_numberOfReallocOperations = 0;
    g_blf_mam_numberOfMemalignOperations = 0;
}

void MemoryAllocationMonitor::endMonitor()
{
    // Disable monitor
    g_blf_mam_monitorEnabled = 0;
}

bool MemoryAllocationMonitor::checkNoMemoryAllocationInLastMonitor()
{
    return (getNumberOfDynamicMemoryOperationsInLastMonitor() == 0);
}

bool MemoryAllocationMonitor::endMonitorAndCheckNoMemoryAllocationInLastMonitor()
{
    endMonitor();
    return checkNoMemoryAllocationInLastMonitor();
}

int32_t MemoryAllocationMonitor::getNumberOfDynamicMemoryOperationsInLastMonitor()
{
    return g_blf_mam_numberOfCallocOperations +
           g_blf_mam_numberOfMallocOperations + 
           g_blf_mam_numberOfFreeOperations +
           g_blf_mam_numberOfReallocOperations +
           g_blf_mam_numberOfMemalignOperations;
}
