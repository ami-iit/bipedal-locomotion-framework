# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_RUN_MemoryAllocationMonitor_tests)
    add_bipedal_test(
      NAME MemoryAllocationMonitor
      SOURCES MemoryAllocationMonitorTest.cpp
      LINKS BipedalLocomotion::TestUtils Eigen3::Eigen
      # As also valgrind uses LD_PRELOAD to override the used malloc,
      # the MemoryAllocationMonitor::getNumberOfDynamicMemoryOperationsInLastMonitor()
      # function always return 0, and MemoryAllocationMonitor::checkNoMemoryAllocationInLastMonitor()
      # always return true when running under valgrind. This is not a problem in normal tests
      # (that tipically test that MemoryAllocationMonitor::checkNoMemoryAllocationInLastMonitor() is true)
      # but make fail the tests of the MemoryAllocationMonitor class, so let's disable them
      DISABLE_VALGRIND_TEST)
endif()
