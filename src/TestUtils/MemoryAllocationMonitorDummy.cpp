/**
 * @file MemoryAllocationMonitorDummy.cpp
 * @authors Silvio Traversaro
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TestUtils/MemoryAllocationMonitor.h>

using namespace BipedalLocomotion::TestUtils;

bool MemoryAllocationMonitor::monitorIsEnabled()
{
    return false;
}

void MemoryAllocationMonitor::startMonitor()
{
    return;
}

void MemoryAllocationMonitor::endMonitor()
{
    return;
}

bool MemoryAllocationMonitor::checkNoMemoryAllocationInLastMonitor()
{
    return true;
}

bool MemoryAllocationMonitor::endMonitorAndCheckNoMemoryAllocationInLastMonitor()
{
    endMonitor();
    return checkNoMemoryAllocationInLastMonitor();
}

int32_t MemoryAllocationMonitor::getNumberOfDynamicMemoryOperationsInLastMonitor()
{
    return 0;
}
