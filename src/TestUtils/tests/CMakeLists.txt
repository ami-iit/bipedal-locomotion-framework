# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_RUN_MemoryAllocationMonitor_tests)
    add_bipedal_test(
      NAME MemoryAllocationMonitor
      SOURCES MemoryAllocationMonitorTest.cpp
      LINKS BipedalLocomotion::TestUtils Eigen3::Eigen)
endif()
